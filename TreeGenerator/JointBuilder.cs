using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace TreeGenerator
{
    class JointBuilder
    {

        private GH_Structure<GH_Plane> inputPlanes = new GH_Structure<GH_Plane>();
        //joint skeleton as curves
        private GH_Structure<GH_Curve> skeletonLines = new GH_Structure<GH_Curve>();
        //parallel layerPlanes for printing
        private GH_Structure<GH_Plane> layerPlanes = new GH_Structure<GH_Plane>();
        //calc intersections between skeleton and layerPlanes
        private GH_Structure<GH_Point> intersections = new GH_Structure<GH_Point>();
        //3d grid for joint
        private GH_Structure<GH_Point> grid = new GH_Structure<GH_Point>();
        //print slices returned as point grid
        private GH_Structure<GH_Point> printSlices_points = new GH_Structure<GH_Point>();
        //print slices returned as meshes
        private GH_Structure<GH_Mesh> printSlices_mesh = new GH_Structure<GH_Mesh>();
        private GH_Structure<GH_Curve> meshBoundaries = new GH_Structure<GH_Curve>();

        private int numX, numY; //number of points per side. 

        private double resolution;
        private double jointRadius;
        private double gridX;
        private double gridY;
        private double layerHeight;

        public GH_Structure<GH_Plane> InputPlanes
        {
            get { return inputPlanes; }
        }
        public GH_Structure<GH_Curve> SkeletonLines
        {
            get { return skeletonLines; }
        }
        public GH_Structure<GH_Plane> LayerPlanes
        {
            get { return layerPlanes; }
        }
        public GH_Structure<GH_Point> Intersections
        {
            get { return intersections; }
        }



        public double Resolution
        {
            get { return resolution; }
            set { resolution = value; }
        }
        public double JointRadius
        {
            get { return jointRadius; }
            set { jointRadius = value; }
        }
        public double GridX
        {
            get { return gridX; }
            set { gridX = value; }
        }
        public double GridY
        {
            get { return gridY; }
            set { gridY = value; }
        }
        public double LayerHeight
        {
            get { return layerHeight; }
            set { layerHeight = value; }
        }


        public JointBuilder(GH_Structure<GH_Plane> inputPlanes_, double resolution_, double gridX_, double gridY_, double jointRadius_, double layerHeight_)
        {
            inputPlanes = inputPlanes_;
            resolution = resolution_;
            gridX = gridX_;
            gridY = gridY_;
            jointRadius = jointRadius_;
            layerHeight = layerHeight_;

            numX = (int)(gridX / resolution);
            numY = (int)(gridY / resolution);

            //double tolerance = T;
            //double maxRange = S + tolerance;
            //double minRange = S - tolerance;
        }

        /// <summary>
        /// Generates all geometry for printing.
        /// </summary>
        public void GeneratePrintLayers()
        {
            //calc curves
            skeletonLines = BuildStructureAsCurves();
            //calc layers
            layerPlanes = BuildPrintLayerPlanes();
            //calc intersections
            intersections = CurvePlaneIntersect();

            //=================== todo code below not tested on multiple joints
            //calc grid
            grid = BuildGrid(intersections, layerPlanes);
            //print slices returned as point grid
            printSlices_points = BuildContourPoints(grid, intersections);

            //build curves from mesh
            printSlices_mesh = BuildMeshSlices(out meshBoundaries);
        }



        /// <summary>
        /// 
        /// </summary>
        /// <param name="layers"></param>
        /// <param name="curves"></param>
        /// <returns></returns>
        private GH_Structure<GH_Point> CurvePlaneIntersect()
        {
            GH_Structure<GH_Point> intersections = new GH_Structure<GH_Point>();

            for (int i = 0; i < Utility.GetMainBranchCount(skeletonLines.Paths); i++)
            {
                int layerCount = 0; //increment per layer for each joint i
                foreach (GH_Plane plane in layerPlanes.get_Branch(i))
                {
                    List<GH_Point> xPts = new List<GH_Point>();
                    for (int j = 0; j < Utility.GetSecondaryBranchCount(skeletonLines.Paths, i); j++)
                    {
                        foreach (GH_Curve c in skeletonLines.get_Branch(new GH_Path(i, j)))
                        {
                            var intevents = Rhino.Geometry.Intersect.Intersection.CurvePlane(c.Value, plane.Value, 0.1); //todo cast prob wont work from GH_Curve to curve
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
            GH_Structure<GH_Point> cleanInts = CleanIntersections(intersections);
            return cleanInts;
        }

        private GH_Structure<GH_Point> CleanIntersections(GH_Structure<GH_Point> intersections)
        {
            GH_Structure<GH_Point> cleanedIntersects = new GH_Structure<GH_Point>();
            for (int i = 0; i < Utility.GetMainBranchCount(intersections.Paths); i++)
            {
                for (int j = 0; j < Utility.GetSecondaryBranchCount(intersections.Paths, i); j++)
                {
                    GH_Path pth = new GH_Path(i, j);
                    List<GH_Point> tempBranch = new List<GH_Point>();
                    for (int k = 0; k < intersections.get_Branch(pth).Count; k++)
                    {
                        if (intersections.get_DataItem(pth, k) != null)
                        {
                            tempBranch.Add(intersections.get_DataItem(pth, k));
                        }
                    }
                    if (tempBranch.Count > 0)
                        cleanedIntersects.AppendRange(tempBranch, pth);
                }
            }

            return cleanedIntersects;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="inputPlanes"></param>
        /// <returns></returns>
        private GH_Structure<GH_Curve> BuildStructureAsCurves()
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


        /// <summary>
        /// 
        /// </summary>
        /// <param name="inputPlanes"></param>
        /// <param name="height"></param>
        /// <returns></returns>
        private GH_Structure<GH_Plane> BuildPrintLayerPlanes()
        {
            GH_Structure<GH_Plane> layers = new GH_Structure<GH_Plane>();
            for (int h = 0; h < Utility.GetMainBranchCount(inputPlanes.Paths); h++)
            {
                List<GH_Plane> layerPlanes = new List<GH_Plane>();

                for (int i = 0; i < 150; i++)
                {
                    GH_Plane ip = inputPlanes.get_DataItem(new GH_Path(h, 0), 0); ;
                    Plane p = new Plane();
                    ip.CastTo<Plane>(out p);
                    Vector3d move = p.ZAxis * layerHeight * i;
                    p.Translate(move); //cannot translate wit GH_Type.Value as it passes by copy
                    GH_Plane op = new GH_Plane();
                    op.CastFrom(p);
                    layerPlanes.Add(op);
                }
                layers.AppendRange(layerPlanes, new GH_Path(h));
            }

            return layers;
        }


        //todo : below only tested in GH component for single joints. might need to restructure to work on all joints. 


        private GH_Structure<GH_Mesh> BuildMeshSlices(out GH_Structure<GH_Curve> meshBoundaries)
        {
            GH_Structure<GH_Mesh> meshLayers = new GH_Structure<GH_Mesh>();
            meshBoundaries = new GH_Structure<GH_Curve>();

            for (int i = 0; i < printSlices_points.PathCount; i++)
            {
                Mesh m = new Mesh();

                Plane lp = Utility.CastToPlane(layerPlanes.get_DataItem(new GH_Path(0), i)); //get layerplane
                int vertexCount = 0;
                foreach (GH_Point pt in printSlices_points.get_Branch(i))
                {
                    Point3d pt3 = Utility.CastToPoint3d(pt);
                    double halfRes = resolution / 2;
                    Point3d v0 = pt3 + (lp.XAxis * halfRes) + (lp.YAxis * halfRes);
                    Point3d v1 = pt3 + (lp.XAxis * halfRes) - (lp.YAxis * halfRes);
                    Point3d v2 = pt3 - (lp.XAxis * halfRes) + (lp.YAxis * halfRes);
                    Point3d v3 = pt3 - (lp.XAxis * halfRes) - (lp.YAxis * halfRes);

                    m.Vertices.Add(v0);
                    vertexCount++;
                    m.Vertices.Add(v1);
                    vertexCount++;
                    m.Vertices.Add(v3);
                    vertexCount++;
                    m.Vertices.Add(v2);
                    vertexCount++;

                    MeshFace mf = new MeshFace(vertexCount - 4, vertexCount - 3, vertexCount - 2, vertexCount - 1);
                    m.Faces.AddFace(mf);
                }
                meshLayers.Append(new GH_Mesh(m), new GH_Path(i));
                var polylines = m.GetNakedEdges();
                foreach (var polyline in polylines)
                {
                    GH_Curve pl = Utility.CastFromPLine(polyline);
                    meshBoundaries.Append(pl, new GH_Path(i));
                }

            }

            return meshLayers;
        }

        public GH_Structure<GH_Point> BuildGrid(GH_Structure<GH_Point> CXP, GH_Structure<GH_Plane> printLayers)
        {
            GH_Structure<GH_Point> grid = new GH_Structure<GH_Point>();

            for (int i = 0; i < CXP.PathCount; i++)
            {
                Plane pl = Utility.CastToPlane(printLayers.get_DataItem(new GH_Path(0), i));

                for (int x = 0; x < numX; x++)
                {
                    for (int y = 0; y < numY; y++)
                    {
                        Point3d p = new Point3d(pl.Origin);
                        Vector3d moveInGrid = new Vector3d();
                        moveInGrid += ((pl.XAxis * gridX / 2 * -1) + (pl.XAxis * x * resolution)) + ((pl.YAxis * gridY / 2 * -1) + (pl.YAxis * y * resolution));
                        p += moveInGrid;
                        grid.Append(new GH_Point(p), new GH_Path(i, x));
                    }
                }
            }

            return grid;
        }

        //public GH_Structure<GH_Point> BuildContourPoints(GH_Structure<GH_Point> grid, GH_Structure<GH_Point> CXP, double maxRange, double minRange)
        //{
        //    GH_Structure<GH_Point> contours = new GH_Structure<GH_Point>();
        //    for (int i = 0; i < Utility.GetMainBranchCount(grid.Paths); i++)
        //    {
        //        HashSet<int> branches2d = GetBranches2D(grid.Paths, i);
        //        for (int j = 0; j < branches2d.Count; j++)
        //        {
        //            for (int k = 0; k < grid.Branch(i, j).Count; k++)
        //            {
        //                bool isUnique = true;
        //                bool isInRange = false;
        //                int overlapCount = 0;
        //                foreach (Point3d intersect in CXP.Branch(0, i))
        //                {
        //                    Point3d p = grid.Branch(i, j)[k];
        //                    double resolution = (p - intersect).Length;
        //                    if (isUnique)
        //                    {
        //                        if (resolution < maxRange && resolution > minRange)
        //                        {
        //                            isUnique = false; //the next time it triggers for another intersect, its not unique anymore
        //                            isInRange = true; //its within tolerance
        //                        }
        //                    }
        //                    if (resolution < minRange) overlapCount++; //if it overlaps with the range of any other intersection point
        //                }
        //                if (overlapCount == 0 && isInRange) contours.Add(grid.Branch(i, j)[k], new GH_Path(i)); //if the point doesn overlap and is within tolerance, add
        //            }
        //        }
        //    }
        //    return contours;
        //}

        public GH_Structure<GH_Point> BuildContourPoints(GH_Structure<GH_Point> grid, GH_Structure<GH_Point> CXP)
        {
            GH_Structure<GH_Point> contours = new GH_Structure<GH_Point>();

            for (int i = 0; i < Utility.GetMainBranchCount(grid.Paths); i++)
            {
                for (int j = 0; j < Utility.GetSecondaryBranchCount(grid.Paths, i); j++)
                {
                    GH_Path pth = new GH_Path(i, j);
                    for (int k = 0; k < grid.get_Branch(pth).Count; k++)
                    {
                        bool isUnique = true;
                        foreach (GH_Point intersect in CXP.get_Branch(new GH_Path(0, i)))
                        {
                            Point3d intPt = Utility.CastToPoint3d(intersect);
                            Point3d p = Utility.CastToPoint3d(grid.get_DataItem(pth, k));
                            double resolution = (p - intPt).Length;
                            if (isUnique)
                            {
                                if (resolution < jointRadius)
                                {
                                    isUnique = false; //the next time it triggers for another intersect, its not unique anymore
                                    contours.Append(grid.get_DataItem(new GH_Path(i, j),k), new GH_Path(i));
                                }
                            }
                        }
                    }
                }
            }
            return contours;
        }

        ///// <summary>
        /////
        ///// </summary>
        ///// <param name="contourPoints"></param>
        ///// <returns></returns>
        //public List<Polyline> GetContourLines(GH_Structure<GH_Point> contourPoints)
        //{
        //    List<Polyline> curves = new List<Polyline>();
        //    for (int i = 0; i < contourPoints.BranchCount; i++) //for each layer
        //    {
        //        Point3dList allPts = new Point3dList(contourPoints.Branch(i));
        //        Polyline l = new Polyline();
        //        Point3d firstPt = contourPoints.Branch(i)[0];

        //        while (allPts.Count != 0)
        //        {

        //            int idx = allPts.ClosestIndex(firstPt);

        //            l.Add(allPts[idx]);
        //            firstPt = allPts[idx];
        //            allPts.RemoveAt(idx);
        //        }

        //        curves.Add(l);
        //    }
        //    return curves;
        //}

    }

}
