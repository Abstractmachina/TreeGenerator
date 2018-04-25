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
            pManager.AddCurveParameter("Tree as Curve", "CV", "Returns tree structure as curves", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("Printing Layers", "L", "Returns planes of printing layers", GH_ParamAccess.tree);
            pManager.AddPointParameter("Tree | Planes intersection", "CXP", "returns intersection events between tree and printing planes.", GH_ParamAccess.tree);

            pManager.AddTextParameter("Debug Log", "BUG", "Debug Log for development", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        /// 
        public List<string> debugLog;
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            debugLog = new List<string>();
            //input
            GH_Structure<GH_Plane> inputPlanes = new GH_Structure<GH_Plane>();
            double height = 0;
            if (!DA.GetDataTree(0, out inputPlanes)) return;
            if (!DA.GetData(1, ref height)) return;

            //calc curves
            GH_Structure<GH_Curve> curves = BuildStructureAsCurves(inputPlanes);
            //calc layers
            GH_Structure<GH_Plane> layers = BuildPrintLayerPlanes(inputPlanes, height);
            //calc intersections
            GH_Structure<GH_Point> intersections = CurvePlaneIntersect(layers, curves);

            //output
            DA.SetDataTree(0, curves);
            DA.SetDataTree(1, layers);
            DA.SetDataTree(2, intersections);
            DA.SetDataList(3, debugLog);
        }

        /// <summary>
        /// Returns all intersection events between a set of curves and printing planes. Resulting data tree is sorted by planes.
        /// </summary>
        /// <param name="layers"></param>
        /// <param name="curves"></param>
        /// <returns></returns>
        private static GH_Structure<GH_Point> CurvePlaneIntersect(GH_Structure<GH_Plane> layers, GH_Structure<GH_Curve> curves)
        {
            GH_Structure<GH_Point> intersections = new GH_Structure<GH_Point>();

            for (int i = 0; i < Utility.GetMainBranchCount(curves.Paths); i++)
            {
                int layerCount = 0; //increment per layer for each joint i
                foreach (GH_Plane plane in layers.get_Branch(i))
                {
                    List<GH_Point> xPts = new List<GH_Point>();
                    for (int j = 0; j < Utility.GetSecondaryBranchCount(curves.Paths, i); j++)
                    {
                        foreach (GH_Curve c in curves.get_Branch(new GH_Path(i, j)))
                        {
                            var intevents = Rhino.Geometry.Intersect.Intersection.CurvePlane(c.Value, plane.Value, 0.1);
                            if (intevents != null)
                            {
                                for (int k = 0; k < intevents.Count; k++)
                                {
                                    var eve = intevents[k];
                                    GH_Point pt = new GH_Point();
                                    pt.CastFrom(eve.PointA);
                                    xPts.Add(pt);
                                }
                            }
                            else xPts.Add(null);
                        }
                    }
                    intersections.AppendRange(xPts, new GH_Path(i, layerCount));
                    layerCount++;
                }
            }

            return intersections;
        }

        private static GH_Structure<GH_Curve> BuildStructureAsCurves(GH_Structure<GH_Plane> inputPlanes)
        {
            GH_Structure<GH_Curve> curves = new GH_Structure<GH_Curve>();

            for (int i = 0; i < Utility.GetMainBranchCount(inputPlanes.Paths); i++)
            {
                int subBranchCount = Utility.GetSecondaryBranchCount(inputPlanes.Paths, i);
                for (int j = 0; j < subBranchCount; j++)
                {
                    GH_Path path = new GH_Path(i, j);
                    List<Point3d> cp = new List<Point3d>();
                    foreach (GH_Plane plane in inputPlanes.get_Branch(path)) cp.Add(plane.Value.Origin);
                    GH_Curve cv = new GH_Curve();
                    cv.CastFrom(Curve.CreateInterpolatedCurve(cp, 3));
                    curves.Append(cv, path);
                }
            }

            return curves;
        }

        private static GH_Structure<GH_Plane> BuildPrintLayerPlanes(GH_Structure<GH_Plane> inputPlanes, double height)
        {
            GH_Structure<GH_Plane> layers = new GH_Structure<GH_Plane>();
            for (int h = 0; h < Utility.GetMainBranchCount(inputPlanes.Paths); h++)
            {
                List<GH_Plane> layerPlanes = new List<GH_Plane>();

                for (int i = 0; i < 100; i++)
                {
                    GH_Plane ip = inputPlanes.get_DataItem(new GH_Path(h, 0), 0); ;
                    Plane p = new Plane();
                    ip.CastTo<Plane>(out p);
                    Vector3d move = p.ZAxis * height * i;
                    p.Translate(move); //cannot translate wit GH_Type.Value as it passes by copy
                    GH_Plane op = new GH_Plane();
                    op.CastFrom(p);
                    layerPlanes.Add(op);
                }
                layers.AppendRange(layerPlanes, new GH_Path(h));
            }

            return layers;
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