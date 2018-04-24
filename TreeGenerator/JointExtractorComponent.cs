using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;

namespace TreeGenerator
{
    public class JointExtractor : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public JointExtractor()
          : base("Joint Extractor", "JoiE",
              "Extracts joints from a branching structure.",
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

            pManager.AddPlaneParameter("Untrimmed Joints", "UT", "Returns all joints as untrimmed list of joints.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("Trimmed Joints", "TT", "Returns all joints based on trim specifications.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("Stems", "S", "Returns stems based on trim specifications", GH_ParamAccess.tree);
            

            pManager.AddTextParameter("Debug Log", "DEBUG", "Debug Log for development.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //input
            GH_Structure<GH_Plane> planes = new GH_Structure<GH_Plane>();
            double t1 = 0;
            double t2 = 0;

            if (!DA.GetDataTree(0, out planes)) return;
            if (!DA.GetData(1, ref t1)) return;
            if (!DA.GetData(2, ref t2)) return;

            int trimLengthBase = (int)t1;
            int trimLengthTop = (int)t2;

            //find joints
            List<List<List<GH_Plane>>> jointsList = ExtractJoints(planes, trimLengthTop);

            //trim joints
            List<List<List<GH_Plane>>> trimmedJointsList = TrimJoints(jointsList, trimLengthBase, trimLengthTop);

            //find stems
            GH_Structure<GH_Plane> stems = ExtractStems(planes, trimLengthBase, trimLengthTop);

            //convert to datatree
            GH_Structure<GH_Plane> untrimmedJointsTree = ConvertListToTree(jointsList);
            GH_Structure<GH_Plane> trimmedJointsTree = ConvertListToTree(trimmedJointsList);

            //output
            DA.SetDataTree(0, untrimmedJointsTree);
            DA.SetDataTree(1, trimmedJointsTree);
            DA.SetDataTree(2, stems);
        }



        /// <summary>
        /// Extract joints from a tree structure with a flat hierarchy. 
        /// </summary>
        /// <param name="planes"></param>
        /// <param name="trimLengthTop"></param>
        /// <returns></returns>
        private List<List<List<GH_Plane>>> ExtractJoints(GH_Structure<GH_Plane> planes, int trimLengthTop)
        {
            List<List<List<GH_Plane>>> jointsList = new List<List<List<GH_Plane>>>(); //3dimensional list to preserve data structure internally

            for (int i = 0; i < planes.PathCount; i++)
            {
                List<List<GH_Plane>> members = new List<List<GH_Plane>>(); //stores members

                //add first member to tree
                List<GH_Plane> member0 = planes.get_Branch(i) as List<GH_Plane>; //cast from Ilist to list
                members.Add(member0);

                for (int j = 0; j < planes.PathCount; j++)
                {
                    if (j != i) //if its not the same branch
                    {
                        //check whether last plane of branch i is the same as first plane of branch j
                        int[] ind = { i, j };
                        List<GH_Plane> otherMember = planes.get_Branch(j) as List<GH_Plane>;
                        Vector3d distVector = (Vector3d)member0[member0.Count - 1].Value.Origin - (Vector3d)otherMember[0].Value.Origin;
                        double dist = distVector.Length;
                        if (dist < 0.01)
                        {
                            //if same plane, add branch
                            members.Add(otherMember);
                        }
                    }
                }

                if (members.Count > 1) //if joint is actually a joint that has more than just a base
                {
                    //check if members have enough planes.
                    //e.g. if a member has only 1 plane, it is not enough to build a joint.
                    bool enoughPlanes = true;
                    for (int k = 0; k < members.Count; k++)
                    {
                        if (members[k].Count < trimLengthTop)
                        {
                            enoughPlanes = false;
                            break;
                        }
                    }
                    if (enoughPlanes) jointsList.Add(members);
                }
            }
            return jointsList;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="jointsList"></param>
        /// <param name="trimLengthBase"></param>
        /// <param name="trimLengthTop"></param>
        /// <returns></returns>
        private List<List<List<GH_Plane>>> TrimJoints(List<List<List<GH_Plane>>> jointsList, int trimLengthBase, int trimLengthTop)
        {
            List<List<List<GH_Plane>>> trimmedJointsList = new List<List<List<GH_Plane>>>();

            for (int i = 0; i < jointsList.Count; i++)
            {
                List<List<GH_Plane>> trimmedJoint = new List<List<GH_Plane>>();

                for (int j = 0; j < jointsList[i].Count; j++)
                {

                    List<GH_Plane> trimmedMember = new List<GH_Plane>();
                    if (j == 0)
                    {
                        for (int k = (jointsList[i][j].Count - trimLengthBase); k < jointsList[i][j].Count; k++)
                        {
                            GH_Plane p = jointsList[i][j][k];
                            trimmedMember.Add(p);
                        }
                    }
                    else
                    {
                        for (int k = 0; k < trimLengthTop; k++)
                        {
                            GH_Plane p = jointsList[i][j][k];
                            trimmedMember.Add(p);
                        }
                    }
                    trimmedJoint.Add(trimmedMember);
                }
                trimmedJointsList.Add(trimmedJoint);
            }

            return trimmedJointsList;
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
        /// 
        /// </summary>
        /// <param name="list"></param>
        /// <returns></returns>
        private GH_Structure<GH_Plane> ConvertListToTree(List<List<List<GH_Plane>>> list)
        {
            GH_Structure<GH_Plane> tree = new GH_Structure<GH_Plane>();
            for (int i = 0; i < list.Count; i++)
            {
                for (int j = 0; j < list[i].Count; j++)
                {
                    GH_Path pth = new GH_Path(i, j);
                    tree.AppendRange(list[i][j], pth);
                }
            }
            return tree;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="planes"></param>
        /// <returns></returns>
        private int GetBranchCount(GH_Structure<GH_Plane> planes)
        {
            int currentBranch = 0;
            int branchSize = 1;
            foreach (GH_Path path in planes.Paths)
            {
                if (path[0] != currentBranch)
                {
                    branchSize++;
                    currentBranch = path[0];
                }
            }
            return branchSize;
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
            get { return new Guid("5bd183aa-733d-4eec-a05c-22b0ca824f7a"); }
        }
    }
}