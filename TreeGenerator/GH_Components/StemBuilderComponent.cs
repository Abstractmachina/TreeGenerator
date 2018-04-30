using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace TreeGenerator
{
    public class StemBuilderComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the StemBuilder class.
        /// </summary>
        public StemBuilderComponent()
          : base("Stem Builder", "SBldr",
              "Builds stems for 3DP based on Normals.",
              "Generative", "Fabrication")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Stems as Curves", "S", "Stem skeleton input", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Radius", "R", "Radii of stems", GH_ParamAccess.item, 15d);
            pManager.AddNumberParameter("Male Joint Height", "JH", "Height of joint that goes into a node", GH_ParamAccess.item, 10d);
            pManager.AddNumberParameter("Male Joint Radius", "JD", "Radius of joint.", GH_ParamAccess.item, 10d);
            pManager.AddNumberParameter("Print Layer Height", "LH", "Layer height of the 3DP.", GH_ParamAccess.item, 2d);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Structure<GH_Curve> inputSkeleton = new GH_Structure<GH_Curve>();
            double radius = 0;
            double jointHeight = 0;
            double jointRadius = 0;
            double layerHeight = 0;

            if (!DA.GetDataTree(0, out inputSkeleton)) return;
            if (!DA.GetData(1, ref radius)) return;
            if (!DA.GetData(2, ref jointHeight)) return;
            if (!DA.GetData(3, ref jointRadius)) return;
            if (!DA.GetData(4, ref layerHeight)) return;

            StemBuilder sb = new StemBuilder(inputSkeleton, radius, jointHeight, jointRadius, layerHeight);


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
            get { return new Guid("2f6e5569-3fc1-444d-ad1b-8f21bff06f4d"); }
        }
    }
}