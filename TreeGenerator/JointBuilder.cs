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
        /***
         * 
         * NOTES:
         * 
         * */


        //initial input planes
        private GH_Structure<GH_Plane> inputPlanes = new GH_Structure<GH_Plane>();
        //joint skeleton as curves
        private GH_Structure<GH_Curve> skeletonLines = new GH_Structure<GH_Curve>();
        //print slices returned as meshes
        private GH_Structure<GH_Mesh> printSlices_mesh = new GH_Structure<GH_Mesh>();

        private int numX, numY; //number of points per side. 

        private double resolution;
        private double jointRadius;
        private double gridX;
        private double gridY;
        private double layerHeight;

        #region Properties

        public GH_Structure<GH_Plane> InputPlanes
        {
            get { return inputPlanes; }
        }
        public GH_Structure<GH_Curve> SkeletonLines
        {
            get { return skeletonLines; }
        }
        
        public GH_Structure<GH_Mesh> PrintSlices_mesh
        {
            get { return printSlices_mesh; }
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
        #endregion

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

        }

        /// <summary>
        /// Generates all geometry for printing. Main functional method.
        /// </summary>
        public void GeneratePrintLayers()
        {
            skeletonLines = BuildStructureAsCurves();
            //
            //iterate through each layer, get intersection, calc point grid, get mesh. 

            //for each joint i

            for (int i = 0; i < Utility.GetMainBranchCount(inputPlanes.Paths); i++) //go through each joint
            {
                Plane currentPlane = Utility.CastToPlane(inputPlanes.get_DataItem(new GH_Path(i, 0), 0));
                bool hasIntersections = true;
                int currentLayer = 1;

                //while there are intersections, keep calculating
                while (hasIntersections)
                {
                    //move plane to currentlayer
                    Vector3d move = currentPlane.ZAxis * layerHeight;
                    currentPlane.Translate(move);
                    //array that holds grid.
                    Point3d[,] tempGrid = GenerateGrid(currentPlane);
                    //calc intersections.
                    List<Point3d> CXP = GenerateIntersections(i, currentPlane);
                    //check whether there are still intersections
                    if (CXP.Count == 0)
                    {
                        hasIntersections = false;
                        break;
                    }

                    //calc printSlices
                    List<Point3d> slicePoints = GenerateSlicePoints(tempGrid, CXP);

                    //build mesh for slice
                    Mesh m = BuildMeshFromSlicePoints(currentPlane, slicePoints);

                    GH_Path pth = new GH_Path(i, currentLayer - 1);
                    printSlices_mesh.Append(new GH_Mesh(m), pth);

                    currentLayer++;

                }
            }
        }

        private Mesh BuildMeshFromSlicePoints(Plane currentPlane, List<Point3d> slicePoints)
        {
            Mesh m = new Mesh();

            int vertexCount = 0;
            foreach (Point3d pt in slicePoints)
            {
                double halfRes = resolution / 2;
                Point3d v0 = pt + (currentPlane.XAxis * halfRes) + (currentPlane.YAxis * halfRes);
                Point3d v1 = pt + (currentPlane.XAxis * halfRes) - (currentPlane.YAxis * halfRes);
                Point3d v2 = pt - (currentPlane.XAxis * halfRes) + (currentPlane.YAxis * halfRes);
                Point3d v3 = pt - (currentPlane.XAxis * halfRes) - (currentPlane.YAxis * halfRes);

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

            return m;
        }

        private List<Point3d> GenerateSlicePoints(Point3d[,] tempGrid, List<Point3d> CXP)
        {
            List<Point3d> slicePoints = new List<Point3d>();

            //go through each grid point and check whether distance to intersection pt is within radius
            foreach (Point3d p in tempGrid)
            {
                bool isUnique = true;
                foreach (Point3d intersect in CXP)
                {
                    double distToCenter = (p - intersect).Length;
                    if (isUnique)
                    {
                        if (distToCenter < jointRadius)
                        {
                            isUnique = false; //the next time it triggers for another intersect, its not unique anymore
                            slicePoints.Add(p);
                        }
                    }
                }
            }

            return slicePoints;
        }

        private List<Point3d> GenerateIntersections(int jointNumber, Plane currentPlane)
        {
            List<Point3d> CXP = new List<Point3d>();
            for (int j = 0; j < Utility.GetSecondaryBranchCount(skeletonLines.Paths, jointNumber); j++)
            {
                foreach (GH_Curve c in skeletonLines.get_Branch(new GH_Path(jointNumber, j)))
                {
                    var intevents = Rhino.Geometry.Intersect.Intersection.CurvePlane(c.Value, currentPlane, 0.01); //todo cast prob wont work from GH_Curve to curve
                    if (intevents != null)
                    {
                        for (int k = 0; k < intevents.Count; k++)
                        {
                            var eve = intevents[k];
                            CXP.Add(eve.PointA);
                        }
                    }
                }
            }

            return CXP;
        }

        private Point3d[,] GenerateGrid(Plane currentPlane)
        {
            Point3d[,] tempGrid = new Point3d[numX, numY];
            //populate grid with points
            for (int x = 0; x < numX; x++)
            {
                for (int y = 0; y < numY; y++)
                {
                    Point3d p = new Point3d(currentPlane.Origin);
                    Vector3d moveInGrid = new Vector3d();
                    moveInGrid += ((currentPlane.XAxis * gridX / 2 * -1) + (currentPlane.XAxis * x * resolution)) + ((currentPlane.YAxis * gridY / 2 * -1) + (currentPlane.YAxis * y * resolution));
                    p += moveInGrid;
                    tempGrid[x, y] = p;
                }
            }

            return tempGrid;
        }

        #region ----------------------------------------------  OLD CODE    ------------------------------------------------------------------------ 
        /// <summary>
        /// Path[0] = joint; Path[1] = layer
        /// </summary>
        /// <param name="layers"></param>
        /// <param name="curves"></param>
        /// <returns></returns>
        private GH_Structure<GH_Point> FindCurvePlaneIntersect(GH_Structure<GH_Plane> layerPlanes)
        {
            GH_Structure<GH_Point> intersections = new GH_Structure<GH_Point>();

            for (int i = 0; i < Utility.GetMainBranchCount(skeletonLines.Paths); i++) //only need to go as far as the last intersection
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
        /// Path[0] = joint; Path[1] = member
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

        /// <summary>
        /// Build a grid for finding print slices in a parallel print. Path[0] = joint; Path[1] = layer; Path[2] = column of points.
        /// </summary>
        /// <returns></returns>
        public GH_Structure<GH_Point> BuildParallelGrid(GH_Structure<GH_Point> intersections, GH_Structure<GH_Plane> layerPlanes)
        {
            GH_Structure<GH_Point> grid = new GH_Structure<GH_Point>();

            for (int i = 0; i < Utility.GetMainBranchCount(intersections.Paths); i++) //iterate through each joint
            {
                for (int j = 0; j < Utility.GetSecondaryBranchCount(intersections.Paths, i); j++) //j is iterating through planes, but using intersection layers, since there are unused ones.
                {
                    Plane pl = Utility.CastToPlane(layerPlanes.get_DataItem(new GH_Path(i), j));

                    for (int x = 0; x < numX; x++)
                    {
                        for (int y = 0; y < numY; y++)
                        {
                            Point3d p = new Point3d(pl.Origin);
                            Vector3d moveInGrid = new Vector3d();
                            moveInGrid += ((pl.XAxis * gridX / 2 * -1) + (pl.XAxis * x * resolution)) + ((pl.YAxis * gridY / 2 * -1) + (pl.YAxis * y * resolution));
                            p += moveInGrid;
                            grid.Append(new GH_Point(p), new GH_Path(i, j, x));
                        }
                    }
                }
            }
            return grid;
        }

     
        /// <summary>
        /// build print slices as points. Path[0] = Joints; Path[1] = layers.
        /// </summary>
        /// <param name="grid"></param>
        /// <param name="CXP"></param>
        /// <returns></returns>
        private GH_Structure<GH_Point> BuildContourPoints(GH_Structure<GH_Point> grid, GH_Structure<GH_Point> intersections)
        {
            GH_Structure<GH_Point> contourPoints = new GH_Structure<GH_Point>();

            for (int i = 0; i < Utility.GetMainBranchCount(grid.Paths); i++) //joint
            {
                for (int j = 0; j < Utility.GetSecondaryBranchCount(grid.Paths, i); j++) //layer
                {
                    for (int k = 0; k < Utility.GetTertiaryBranchCount(grid.Paths, i, j); k++) //column
                    {
                        GH_Path pth = new GH_Path(i, j, k);
                        for (int l = 0; l < grid.get_Branch(pth).Count; l++)
                        {
                            bool isUnique = true;
                            foreach (GH_Point intersect in intersections.get_Branch(new GH_Path(i, j)))
                            {
                                Point3d intPt = Utility.CastToPoint3d(intersect);
                                Point3d p = Utility.CastToPoint3d(grid.get_DataItem(pth, l));
                                double distToCenter = (p - intPt).Length;
                                if (isUnique)
                                {
                                    if (distToCenter < jointRadius)
                                    {
                                        isUnique = false; //the next time it triggers for another intersect, its not unique anymore
                                        contourPoints.Append(grid.get_DataItem(pth, l), new GH_Path(i, j));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return contourPoints;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private GH_Structure<GH_Mesh> BuildMeshSlices(GH_Structure<GH_Point> printSlices_points, GH_Structure<GH_Plane> layerPlanes)
        {
            GH_Structure<GH_Mesh> meshLayers = new GH_Structure<GH_Mesh>();
            GH_Structure <GH_Curve> meshBoundaries = new GH_Structure<GH_Curve>();

            for (int i = 0; i < Utility.GetMainBranchCount(printSlices_points.Paths); i++) //joint
            {
                Plane lp = Utility.CastToPlane(layerPlanes.get_DataItem(new GH_Path(i), 0)); //get first layerplane of each joint
                for (int j = 0; j < Utility.GetSecondaryBranchCount(printSlices_points.Paths, i); j++) //layer
                {
                    Mesh m = new Mesh();

                    GH_Path pth = new GH_Path(i, j);

                    int vertexCount = 0;
                    foreach (GH_Point pt in printSlices_points.get_Branch(pth))
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
                    meshLayers.Append(new GH_Mesh(m), pth);
                    var polylines = m.GetNakedEdges();
                    foreach (var polyline in polylines)
                    {
                        GH_Curve pl = Utility.CastFromPLine(polyline);
                        meshBoundaries.Append(pl, pth);
                    }
                }

            }
            return meshLayers;
        }
        #endregion
    }

}
