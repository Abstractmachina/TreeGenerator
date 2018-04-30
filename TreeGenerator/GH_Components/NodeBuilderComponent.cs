using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace TreeGenerator
{
    public class NodeBuilderComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the JointBuilderComponent class.
        /// </summary>
        public NodeBuilderComponent()
          : base("Node Builder", "JoiB",
              "Builds a 3d-printable node.",
              "Generative", "Fabrication")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("node Planes", "JOI", "node from node Extractor Component.", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Layer Height", "H", "Layer height for printing", GH_ParamAccess.item, 5d);
            pManager.AddNumberParameter("Grid Resolution", "RES", "Resolution of builder grid", GH_ParamAccess.item, 2d);
            pManager.AddNumberParameter("Grid Width", "XDIM", "Width of builder grid", GH_ParamAccess.item, 200d);
            pManager.AddNumberParameter("Grid Depth", "YDIM", "Depth of builder grid", GH_ParamAccess.item, 200d);
            pManager.AddNumberParameter("node Radius", "JR", "Radious of resulting node.", GH_ParamAccess.item, 15d);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Slices as Mesh", "SLM", "Slices as Mesh objects.", GH_ParamAccess.tree);

            //DEBUG
            pManager.AddTextParameter("Debug Log", "BUG", "Debug Log for development", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        /// 
        public static List<string> debugLog;
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            NodeBuilder builder;
            debugLog = new List<string>();
            //input
            GH_Structure<GH_Plane> inputPlanes = new GH_Structure<GH_Plane>();
            double layerHeight = 0;
            double resolution = 0;
            double xDim = 0;
            double yDim = 0;
            double radius = 0;
            if (!DA.GetDataTree(0, out inputPlanes)) return;
            if (!DA.GetData(1, ref layerHeight)) return;
            if (!DA.GetData(2, ref resolution)) return;
            if (!DA.GetData(3, ref xDim)) return;
            if (!DA.GetData(4, ref yDim)) return;
            if (!DA.GetData(5, ref radius)) return;

            builder = new NodeBuilder(inputPlanes, resolution, xDim, yDim, radius, layerHeight);

            builder.GeneratePrintLayers();

            //output
            DA.SetDataTree(0, builder.PrintSlices_mesh);
            DA.SetDataList(1, builder.DebugLog);
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
            get { return new Guid("0c97c1f4-f8c7-4eda-a932-a5d062cf263f"); }
        }
    }
}