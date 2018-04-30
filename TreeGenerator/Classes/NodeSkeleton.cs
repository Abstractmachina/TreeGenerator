using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TreeGenerator
{
    class NodeSkeleton
    {
        //INPUT
        private GH_Structure<GH_Plane> inputPlanes;
        private double trimLengthBase;
        private double trimLengthTop;
        private Mesh boundary;


        //OUTPUT
        private GH_Structure<GH_Curve> skeleton = new GH_Structure<GH_Curve>();
        private GH_Structure<GH_Curve> untrimmedNodes = new GH_Structure<GH_Curve>();
        private GH_Structure<GH_Curve> trimmedNodes = new GH_Structure<GH_Curve>();
        private GH_Structure<GH_Curve> stems = new GH_Structure<GH_Curve>();

        private List<Point3d> trimPoints;

        public List<string> DebugLog;

        public GH_Structure<GH_Curve> Skeleton
        {
            get { return skeleton; }
        }
        public GH_Structure<GH_Curve> UntrimmedNodes
        {
            get { return untrimmedNodes; }
        }
        public GH_Structure<GH_Curve> TrimmedNodes
        {
            get { return trimmedNodes; }
        }
        public GH_Structure<GH_Curve> Stems
        {
            get { return stems; }
        }
        public List<Point3d> TrimPoints
        {
            get { return trimPoints; }
        }


        public NodeSkeleton(GH_Structure<GH_Plane> inputPlanes_, double trimLengthBase_, double trimLengthTop_, Mesh boundary_)
        {
            inputPlanes = inputPlanes_;
            trimLengthBase = trimLengthBase_;
            trimLengthTop = trimLengthTop_;
            boundary = boundary_;
            DebugLog = new List<string>();
        }


        public void GenerateNodeSkeleton()
        {
            trimPoints = new List<Point3d>();

            //build curves
            skeleton = BuildStructureAsCurves(inputPlanes);

            //extract nodes
            untrimmedNodes = ExtractNodes(skeleton, trimLengthBase, trimLengthTop);
            //trim nodes
            trimmedNodes = TrimNodes(untrimmedNodes, trimLengthBase, trimLengthTop, out trimPoints);
            //find stems
            stems = ExtractStems(skeleton, trimLengthBase, trimLengthTop, trimPoints);
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
        /// Extract nodes from a tree structure with a flat hierarchy and returns grouped as nodes. NOTE: will have duplicate lines, as untrimmed. 
        /// </summary>
        /// <param name="skeleton"></param>
        /// <param name="trimLengthBase"></param>
        /// <param name="trimLengthTop"></param>
        /// <returns></returns>
        private GH_Structure<GH_Curve> ExtractNodes(GH_Structure<GH_Curve> skeleton, double trimLengthBase, double trimLengthTop)
        {
            GH_Structure<GH_Curve> nodes = new GH_Structure<GH_Curve>(); //3dimensional list to preserve data structure internally
            int nodeCount = 0;

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

                if (tempMembers.Count > 1) //if node is actually a node that is more than just a base
                {
                    //check if members have enough planes.
                    //e.g. if a member has only 1 plane, it is not enough to build a node.
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
                        nodes.AppendRange(tempMembers, new GH_Path(nodeCount));
                        nodeCount++;
                    }
                }
            }
            return nodes;
        }

        private GH_Structure<GH_Curve> TrimNodes(GH_Structure<GH_Curve> untrimmedNodes, double trimLengthBase, double trimLengthTop, out List<Point3d> trimPoints)
        {
            GH_Structure<GH_Curve> trimmedNodes = new GH_Structure<GH_Curve>();
            trimPoints = new List<Point3d>();


            for (int i = 0; i < untrimmedNodes.PathCount; i++)
            {
                GH_Path pth = new GH_Path(i);
                for (int j = 0; j < untrimmedNodes.get_Branch(i).Count; j++)
                {
                    if (j == 0)
                    {
                        GH_Curve crv = untrimmedNodes.get_DataItem(pth, j);
                        double discardLength = crv.Value.GetLength() - trimLengthBase;
                        double t = 0;
                        crv.Value.LengthParameter(discardLength, out t);
                        Point3d splitpt = crv.Value.PointAt(t);
                        trimPoints.Add(splitpt);
                        Curve[] splits = crv.Value.Split(t);
                        trimmedNodes.Append(new GH_Curve(splits[1]), pth);

                    }
                    else
                    {
                        GH_Curve crv = untrimmedNodes.get_DataItem(pth, j);
                        double t = 0;
                        crv.Value.LengthParameter(trimLengthTop, out t);
                        Point3d splitpt = crv.Value.PointAt(t);
                        trimPoints.Add(splitpt);
                        Curve[] splits = crv.Value.Split(t);
                        trimmedNodes.Append(new GH_Curve(splits[0]), pth);
                    }
                }
            }
            return trimmedNodes;
        }

        private GH_Structure<GH_Curve> ExtractStems(GH_Structure<GH_Curve> skeleton, double trimLengthBase, double trimLengthTop, List<Point3d> trimPts)
        {
            GH_Structure<GH_Curve> stems = new GH_Structure<GH_Curve>();
            List<GH_Curve> baseStems = new List<GH_Curve>();
            List<GH_Curve> endStems = new List<GH_Curve>();
            List<GH_Curve> centerStems = new List<GH_Curve>();
            List<GH_Curve> culledStems = new List<GH_Curve>();

            for (int i = 0; i < skeleton.PathCount; i++)
            {
                GH_Path pth = new GH_Path(i);
                GH_Curve crv = skeleton.get_DataItem(pth, 0);



                if (crv.Value.PointAtStart.Z == 0) //the base stems
                {
                    double t = 0;
                    crv.Value.LengthParameter((crv.Value.GetLength() - trimLengthBase), out t);
                    Curve[] splits = crv.Value.Split(t);
                    baseStems.Add(new GH_Curve(splits[0]));
                    
                }
                else
                {

                    double distBoundary = (crv.Value.PointAtEnd - boundary.ClosestPoint(crv.Value.PointAtEnd)).Length;
                    if (distBoundary > 2 && crv.Value.GetLength() > trimLengthBase + trimLengthTop) //stems that are not ends and are long enough
                    {
                        bool hasTopNode = false;
                        foreach (Point3d p in trimPoints)
                        {
                            double distCrvEnds = (p - crv.Value.PointAtEnd).Length;
                            if (distCrvEnds > trimLengthBase -0.1 && distCrvEnds < trimLengthBase + 0.1)
                            {
                                hasTopNode = true;
                                break;
                            }
                        }
                        if (hasTopNode)
                        {
                            List<double> t = new List<double>();
                            double a = 0;
                            crv.Value.LengthParameter(trimLengthTop, out a);
                            t.Add(a);
                            double b = 0;
                            crv.Value.LengthParameter((crv.Value.GetLength() - trimLengthBase), out b);
                            t.Add(b);
                            Curve[] splits = crv.Value.Split(t);
                            centerStems.Add(new GH_Curve(splits[1]));
                            
                        }
                        else
                        {
                            double t = 0;
                            crv.Value.LengthParameter(trimLengthTop, out t);
                            Curve[] splits = crv.Value.Split(t);
                            endStems.Add(new GH_Curve(splits[1]));
                        }
                    }
                    else if (distBoundary < 2 && crv.Value.GetLength() > trimLengthTop) //stems that touch boundary, thus are end stems and that are long enough to be trimmed
                    {
                        double t = 0;
                        crv.Value.LengthParameter(trimLengthTop, out t);
                        Curve[] splits = crv.Value.Split(t);
                        endStems.Add(new GH_Curve(splits[1]));
                        
                    }
                    else //all remaining branches
                    {
                        culledStems.Add(crv);
                    }
                }
            }
            GH_Path basePth = new GH_Path(0);
            GH_Path centerPth = new GH_Path(1);
            GH_Path endPth = new GH_Path(2);
            GH_Path culledPth = new GH_Path(3);

            stems.AppendRange(baseStems, basePth);
            stems.AppendRange(centerStems, centerPth);
            stems.AppendRange(endStems, endPth);
            stems.AppendRange(culledStems, culledPth);

            return stems;
        }
    }
}
