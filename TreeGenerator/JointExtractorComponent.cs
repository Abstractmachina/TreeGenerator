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
              "Generative", "Agent-based")
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

            pManager.AddTextParameter("Debug Log", "DEBUG", "Debug Log for development.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Structure<GH_Plane> planes = new GH_Structure<GH_Plane>();
            int trimLengthBase = 5;
            int trimLengthTop = 5;


            if (!DA.GetDataTree(0, out planes)) return;
            if (!DA.GetData<int>(1, ref trimLengthBase)) return;
            if (!DA.GetData<int>(2, ref trimLengthTop)) return;


            GH_Structure<GH_Plane> joints = new GH_Structure<GH_Plane>();

            List<List<List<Plane>>> jointsList = new List<List<List<Plane>>>(); //3dimensional list to preserve data structure.
                                                                                //datatree cant do it. why??
                                                                                //datatree is a sortedList. uses key-value pairs for indexing.
                                                                                //ie its only 2 dimensional.


            int jointCount = 0;

            for (int i = 0; i < planes.PathCount; i++)
            {
                //List<List<Plane>> members = new List<List<Plane>>(); //stores members

                //add first member to tree

                //for (int j=0;j<planes.get_Branch(i).Count; j++)
                //{
                //    Plane p = new Plane();
                //    planes.get_DataItem(new GH_Path(i), j).CastTo<Plane>(out p);

                //}
                List<GH_Plane> member0 = planes.get_Branch(i).;
                int[] pathDimensions = new int[2];
                GH_Path path = new GH_Path(pathDimensions[0])
                members.Add(member0);


                for (int j = 0; j < planes.BranchCount; j++)
                {
                    if (j != i) //if its not the same branch
                    {
                        //check whether last plane of branch i is the same as first plane of branch j
                        List<Plane> otherMember = planes.Branch(j);
                        Vector3d distVector = (Vector3d)member0[member0.Count - 1].Origin - (Vector3d)otherMember[0].Origin;
                        double dist = distVector.Length;
                        if (dist < 0.01)
                        {
                            //if same plane, add branch
                            members.Add(otherMember);
                        }
                    }
                }


                if (members.Count > 1)
                {

                    //check if members have enough planes.
                    //e.g. if a member has only 1 plane, it is not enough to build a joint.
                    bool enoughPlanes = true;
                    for (int k = 0; k < members.Count; k++)
                    {
                        if (members[k].Count < trimLengthChild)
                        {
                            enoughPlanes = false;
                            break;
                        }
                    }


                    if (enoughPlanes)
                    {
                        jointsList.Add(members);
                        for (int k = 0; k < members.Count; k++)
                        {
                            GH_Path jointPth = new GH_Path(jointCount, k);

                            joints.AddRange(members[k], jointPth);
                        }
                        jointCount++;
                    }
                }
            }

            /*
            for (int i = 0; i < jointsList.Count; i++)
            {
              for (int j = 0; j < jointsList[i].Count; j++)
              {
                Print("joint " + i + ", member " + j + ", number of items: " + jointsList[i][j].Count.ToString());
              }
            }

          */

            //trim joints
            //joint< Member<Planes> >

            //method 2
            List<List<List<Plane>>> trimmedJointsList = new List<List<List<Plane>>>();

            for (int i = 0; i < jointsList.Count; i++)
            {
                List<List<Plane>> trimmedJoint = new List<List<Plane>>();

                for (int j = 0; j < jointsList[i].Count; j++)
                {

                    List<Plane> trimmedMember = new List<Plane>();
                    if (j == 0)
                    {
                        for (int k = (jointsList[i][j].Count - trimLengthParent); k < jointsList[i][j].Count; k++)
                        {
                            Plane p = jointsList[i][j][k];
                            trimmedMember.Add(p);
                        }
                    }
                    else
                    {
                        for (int k = 0; k < trimLengthChild; k++)
                        {
                            Plane p = jointsList[i][j][k];
                            trimmedMember.Add(p);
                        }
                    }
                    trimmedJoint.Add(trimmedMember);
                }
                trimmedJointsList.Add(trimmedJoint);
            }


            for (int i = 0; i < trimmedJointsList.Count; i++)
            {
                for (int j = 0; j < trimmedJointsList[i].Count; j++)
                {
                    Print("joint " + i + ", member " + j + ", number of items: " + trimmedJointsList[i][j].Count.ToString());
                }
            }

            //convert to datatree

            DataTree<Plane> trimmedJointsTree = new DataTree<Plane>();
            for (int i = 0; i < trimmedJointsList.Count; i++)
            {
                for (int j = 0; j < trimmedJointsList[i].Count; j++)
                {
                    GH_Path pth = new GH_Path(i, j);
                    trimmedJointsTree.AddRange(trimmedJointsList[i][j], pth);
                }
            }


            //Print(jointsList.Count.ToString());
            //Print(jointsList[0].Count.ToString());
            //Print(joints.BranchCount.ToString());
            //Print(joints.Branch(0).Count.ToString());

            A = joints;
            B = trimmedJointsTree;

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