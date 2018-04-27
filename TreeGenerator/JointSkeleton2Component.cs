using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;

namespace TreeGenerator
{
    public class JointSkeleton2 : GH_Component
    {
        public JointSkeleton2()
          : base("Joint Skeleton CRV", "JSkeletCRV",
              "Extracts skeleton for joints from a branching structure, curve version.",
              "Generative", "Fabrication")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Tree Planes", "TrPl", "planes describing the structure of a tree, ordered into two dimensional datatrees ( e.g. branch{0}, branch {1} etc).", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Trim Length Base", "TrimB", "Length of joint base", GH_ParamAccess.item, 5d);
            pManager.AddNumberParameter("Trim Length Top", "TrimT", "Length of joint tops", GH_ParamAccess.item, 5d);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {

            pManager.AddCurveParameter("Untrimmed Joints", "UJ", "Returns all joints as untrimmed tree of joints.", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Trimmed Joints", "TJ", "Returns all joints based on trim specifications.", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Stems", "S", "Returns stems based on trim specifications", GH_ParamAccess.tree);
            pManager.AddPointParameter("Trim Points", "TPts", "Trim Points", GH_ParamAccess.list);

            pManager.AddTextParameter("Debug Log", "DEBUG", "Debug Log for development.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        public static List<string> debugLog;
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //input
            GH_Structure<GH_Plane> planes = new GH_Structure<GH_Plane>();
            double trimLengthBase = 0;
            double trimLengthTop = 0;
            debugLog = new List<string>();

            if (!DA.GetDataTree(0, out planes)) return;
            if (!DA.GetData(1, ref trimLengthBase)) return;
            if (!DA.GetData(2, ref trimLengthTop)) return;

            List<Point3d> trimPts = new List<Point3d>();

            //build curves
            GH_Structure<GH_Curve> skeletonLines = BuildStructureAsCurves(planes);

            //extract joints
            GH_Structure<GH_Curve> untrimmedJoints = ExtractJoints(skeletonLines, trimLengthBase, trimLengthTop);
            //trim joints
            GH_Structure<GH_Curve> trimmedJoints = TrimJoints(untrimmedJoints, trimLengthBase, trimLengthTop, out trimPts);
            //find stems
            GH_Structure<GH_Curve> stems = ExtractStems(skeletonLines, trimLengthBase, trimLengthTop, trimPts);


            //output
            DA.SetDataTree(0, untrimmedJoints);
            DA.SetDataTree(1, trimmedJoints);
            DA.SetDataTree(2, stems);
            DA.SetDataList(3, trimPts);
            DA.SetDataList(4, debugLog);
        }

        /// <summary>
        /// Build curves representation with 1D tree structure. branch = one curve
        /// </summary>
        /// <param name="inputPlanes"></param>
        /// <returns></returns>
        private GH_Structure<GH_Curve> BuildStructureAsCurves(GH_Structure<GH_Plane> inputPlanes)
        {
            GH_Structure<GH_Curve> curves = new GH_Structure<GH_Curve>();

            for (int i = 0; i < inputPlanes.PathCount; i++)
            {
                List<Point3d> cp = new List<Point3d>();
                foreach (GH_Plane plane in inputPlanes.get_Branch(i)) cp.Add(plane.Value.Origin);
                GH_Curve cv = new GH_Curve();
                cv.CastFrom(Curve.CreateInterpolatedCurve(cp, 3));
                curves.Append(cv, new GH_Path(i));
            }
            return curves;
        }

        /// <summary>
        /// Extract joints from a tree structure with a flat hierarchy and returns grouped as joints. NOTE: will have duplicate lines, as untrimmed. 
        /// </summary>
        /// <param name="skeleton"></param>
        /// <param name="trimLengthBase"></param>
        /// <param name="trimLengthTop"></param>
        /// <returns></returns>
        private GH_Structure<GH_Curve> ExtractJoints(GH_Structure<GH_Curve> skeleton, double trimLengthBase, double trimLengthTop)
        {
            GH_Structure<GH_Curve> joints = new GH_Structure<GH_Curve>(); //3dimensional list to preserve data structure internally
            int jointCount = 0;

            for (int i = 0; i < skeleton.PathCount; i++)
            {
                List<GH_Curve> tempMembers = new List<GH_Curve>();
                //add first member to tree

                GH_Curve member0 = skeleton.get_DataItem(new GH_Path(i), 0);
                tempMembers.Add(member0);

                for (int j = 0; j < skeleton.PathCount; j++)
                {
                    if (j != i) //if its not the same branch
                    {
                        //check whether last plane of branch i is the same as first plane of branch j

                        GH_Curve otherMember = skeleton.get_DataItem(new GH_Path(j), 0);
                        Vector3d distVector = member0.Value.PointAtEnd - otherMember.Value.PointAtStart;
                        double dist = distVector.Length;
                        if (dist < 0.01)
                        {
                            //if same plane, add branch
                            tempMembers.Add(otherMember);
                        }
                    }
                }

                if (tempMembers.Count > 1) //if joint is actually a joint that is more than just a base
                {
                    //check if members have enough planes.
                    //e.g. if a member has only 1 plane, it is not enough to build a joint.
                    bool isSufficient = true;
                    foreach (GH_Curve crv in tempMembers)
                    {
                        if (crv.Value.GetLength() < trimLengthTop)
                        {
                            isSufficient = false;
                            break;
                        }
                    }
                    if (isSufficient)
                    {
                        joints.AppendRange(tempMembers, new GH_Path(jointCount));
                        jointCount++;
                    }
                }
            }
            return joints;
        }

        private GH_Structure<GH_Curve> TrimJoints(GH_Structure<GH_Curve> untrimmedJoints, double trimLengthBase, double trimLengthTop, out List<Point3d> trimPoints)
        {
            GH_Structure<GH_Curve> trimmedJoints = new GH_Structure<GH_Curve>();
            trimPoints = new List<Point3d>();

            for (int i = 0; i < untrimmedJoints.PathCount; i++)
            {
                GH_Path pth = new GH_Path(i);
                for (int j = 0; j < untrimmedJoints.get_Branch(i).Count; j++)
                {
                    if (j == 0)
                    {
                        GH_Curve crv = untrimmedJoints.get_DataItem(pth, j);
                        double discardLength = crv.Value.GetLength() - trimLengthBase;
                        double t = 0;
                        crv.Value.LengthParameter(discardLength, out t);
                        Point3d splitpt = crv.Value.PointAt(t);
                        trimPoints.Add(splitpt);
                        Curve[] splits = crv.Value.Split(t);
                        trimmedJoints.Append(new GH_Curve(splits[1]), pth);

                    }
                    else
                    {
                        GH_Curve crv = untrimmedJoints.get_DataItem(pth, j);
                        double t = 0;
                        crv.Value.LengthParameter(trimLengthTop, out t);
                        Point3d splitpt = crv.Value.PointAt(t);
                        trimPoints.Add(splitpt);
                        Curve[] splits = crv.Value.Split(t);
                        trimmedJoints.Append(new GH_Curve(splits[0]), pth);
                    }
                }
            }
            return trimmedJoints;
        }

        private GH_Structure<GH_Curve> ExtractStems(GH_Structure<GH_Curve> skeleton, double trimLengthBase, double trimLengthTop, List<Point3d> trimPts)
        {
            GH_Structure<GH_Curve> stems = new GH_Structure<GH_Curve>();

            //find all branches that were removed due to shortness
            List<GH_Curve> removedBranches = new List<GH_Curve>();
            foreach (GH_Curve c in skeleton) if (c.Value.GetLength() < trimLengthTop) removedBranches.Add(c);



            for (int i = 0; i < skeleton.PathCount; i++)
            {
                GH_Path pth = new GH_Path(i);
                GH_Curve crv = skeleton.get_DataItem(pth, 0);



                if (crv.Value.PointAtStart.Z == 0) //the base stems
                {
                    double t = 0;
                    crv.Value.LengthParameter((crv.Value.GetLength() - trimLengthBase), out t);
                    Curve[] splits = crv.Value.Split(t);

                    stems.Append(new GH_Curve(splits[0]), pth);
                }
                else
                {

                    bool hasBothEnds = true;
                    int hits = 0;
                    foreach (Point3d p in trimPts)
                    {
                        double dist1 = (p - crv.Value.PointAtEnd).Length;
                        double dist2 = (p - crv.Value.PointAtStart).Length;
                        if (dist1 < 0.01 || dist2 < 0.01)
                        {
                            hits++;
                        }
                    }
                    if (hits != 2) hasBothEnds = false;

                    if (crv.Value.GetLength() > trimLengthBase + trimLengthTop && hasBothEnds) //all stems that have a joint at the start and end
                    {
                        List<double> t = new List<double>();
                        double a = 0;
                        crv.Value.LengthParameter(trimLengthTop, out a);
                        t.Add(a);
                        double b = 0;
                        crv.Value.LengthParameter((crv.Value.GetLength() - trimLengthBase), out b);
                        t.Add(b);
                        Curve[] splits = crv.Value.Split(t);
                        stems.Append(new GH_Curve(splits[1]), pth);
                    }
                    else if (crv.Value.GetLength() > trimLengthBase + trimLengthTop && !hasBothEnds)//the rest?
                    {

                        double t = 0;
                        crv.Value.LengthParameter(trimLengthTop, out t);
                        Curve[] splits = crv.Value.Split(t);

                        stems.Append(new GH_Curve(splits[1]), pth);
                    }

                }
            }
            return stems;
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="planes"></param>
        /// <param name="trimLengthBase"></param>
        /// <param name="trimLengthTop"></param>
        /// <returns></returns>
        private GH_Structure<GH_Plane> ExtractStems(GH_Structure<GH_Plane> planes, int trimLengthBase, int trimLengthTop)
        {
            GH_Structure<GH_Plane> stems = new GH_Structure<GH_Plane>();
            for (int i = 0; i < planes.PathCount; i++)
            {
                GH_Plane plane = planes.get_DataItem(planes.get_Path(i), 0);
                List<GH_Plane> stemPlanes = new List<GH_Plane>();

                if (plane.Value.OriginZ == 0)
                {
                    for (int j = 0; j < planes.get_Branch(i).Count - trimLengthBase + 1; j++)
                    {
                        stemPlanes.Add(planes.get_DataItem(planes.get_Path(i), j));
                    }
                }
                else
                {
                    for (int j = trimLengthTop - 1; j < planes.get_Branch(i).Count - trimLengthBase + 1; j++)
                    {

                        stemPlanes.Add(planes.get_DataItem(planes.get_Path(i), j));
                    }
                }
                GH_Path path = new GH_Path(i);
                stems.AppendRange(stemPlanes, path);
            }

            return stems;
        }


        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("29687793-02fd-4c7b-890c-01df1d871bc2"); }
        }
    }
}