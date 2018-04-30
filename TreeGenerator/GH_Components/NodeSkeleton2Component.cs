using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;

namespace TreeGenerator
{
    public class NodeSkeleton2Component : GH_Component
    {
        public NodeSkeleton2Component()
          : base("Node Skeleton CRV", "JSkeletCRV",
              "Extracts skeleton for nodes from a branching structure, curve version.",
              "Generative", "Fabrication")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Tree Planes", "TrPl", "planes describing the structure of a tree, ordered into two dimensional datatrees ( e.g. branch{0}, branch {1} etc).", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Trim Length Base", "TrimB", "Length of node base", GH_ParamAccess.item, 5d);
            pManager.AddNumberParameter("Trim Length Top", "TrimT", "Length of node top", GH_ParamAccess.item, 5d);
            pManager.AddMeshParameter("Mesh Boundary", "Mesh", "Mesh Boundary of Geometry", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {

            pManager.AddCurveParameter("Untrimmed nodes", "UJ", "Returns all nodes as untrimmed tree of nodes.", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Trimmed nodes", "TJ", "Returns all nodes based on trim specifications.", GH_ParamAccess.tree);
            pManager.AddCurveParameter("Stems", "S", "Returns stems based on trim specifications", GH_ParamAccess.tree);
            pManager.AddPointParameter("Trim Points", "TPts", "Trim Points", GH_ParamAccess.list);

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
            double trimLengthBase = 0;
            double trimLengthTop = 0;
            Mesh boundary = new Mesh();

            if (!DA.GetDataTree(0, out planes)) return;
            if (!DA.GetData(1, ref trimLengthBase)) return;
            if (!DA.GetData(2, ref trimLengthTop)) return;
            if (!DA.GetData(3, ref boundary)) return;

            NodeSkeleton skel = new NodeSkeleton(planes, trimLengthBase, trimLengthTop, boundary);
            skel.GenerateNodeSkeleton();

            //output
            DA.SetDataTree(0, skel.UntrimmedNodes);
            DA.SetDataTree(1, skel.TrimmedNodes);
            DA.SetDataTree(2, skel.Stems);
            DA.SetDataList(3, skel.TrimPoints);
            DA.SetDataList(4, skel.DebugLog);
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