using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace TreeGenerator
{
    public class JointBuilderComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the JointBuilderComponent class.
        /// </summary>
        public JointBuilderComponent()
          : base("Joint Builder", "JoiB",
              "Builds a 3d-printable joint",
              "Generative", "Fabrication")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Joint Planes", "Joints", "Joints from Joint Extractor Component.", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Layer Height", "Height", "Layer height for printing", GH_ParamAccess.item, 5d);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            
            pManager.AddTextParameter("Debug Log", "BUG", "Debug Log for development", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Structure<GH_Plane> inputPlanes = new GH_Structure<GH_Plane>();
            double height = 0;
            if (!DA.GetDataTree(0, out inputPlanes)) return;
            if (!DA.GetData(1, ref height)) return;

            //calc layers
            int mainBranchCount = Utility.GetMainBranchCount(inputPlanes.Paths);


            GH_Structure<GH_Plane> layers = new GH_Structure<GH_Plane>();
            for (int h = 0; h < mainBranchCount; h++)
            {
                List<GH_Plane> layerPlanes = new List<GH_Plane>();

                for (int i = 0; i < N; i++)
                {
                    GH_Plane p = inputPlanes.get_DataItem(new GH_Path(h, 0), 0);
                    Vector3d move = p.Value.ZAxis * height * i;
                    p.Value.Translate(move);
                    layerPlanes.Add(p);
                }
                layers.AppendRange(layerPlanes, new GH_Path(h));
            }
            //LP = layers;


            //calc curves
            GH_Structure<GH_Curve> curves = new GH_Structure<GH_Curve>();

            for (int i = 0; i < mainBranchCount; i++)
            {
                int subBranchCount = Utility.GetSecondaryBranchCount(inputPlanes.Paths, i);
                for (int j = 0; j < subBranchCount; j++)
                {
                    GH_Path path = new GH_Path(i, j);
                    List<Point3d> cp = new List<Point3d>();
                    foreach (Plane plane in inputPlanes.Branch(i, j)) cp.Add(plane.Origin);
                    Curve cv = Curve.CreateInterpolatedCurve(cp, 3);
                    curves.Add(cv, path);
                }
            }
            //CRV = curves;


            //calc intersections
            GH_Structure<GH_Point> intersections = new GH_Structure<GH_Point>();


            for (int i = 0; i < mainBranchCount; i++)
            {
                int layerCount = 0;
                foreach (Plane plane in layers.Branch(i))
                {
                    List<Point3d> xPts = new List<Point3d>();
                    for (int j = 0; j < Utility.GetSecondaryBranchCount(curves.Paths, i); j++)
                    {
                        foreach (Curve c in curves.Branch(i, j))
                        {
                            var intevents = Rhino.Geometry.Intersect.Intersection.CurvePlane(c, plane, 0.1);
                            if (intevents != null)
                            {
                                for (int k = 0; k < intevents.Count; k++)
                                {
                                    var eve = intevents[k];
                                    xPts.Add(eve.PointA);
                                }
                            }
                        }
                    }
                    intersections.AddRange(xPts, new GH_Path(i, layerCount));
                    layerCount++;
                }
            }
            CXP = intersections;
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