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
    class NodeBuilder
    {
        #region Properties

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
            get { return nodeRadius; }
            set { nodeRadius = value; }
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
        //debug
        public List<string> DebugLog = new List<string>();
        public GH_Structure<GH_Point> printSlices_points = new GH_Structure<GH_Point>();

        //initial input planes
        private GH_Structure<GH_Plane> inputPlanes = new GH_Structure<GH_Plane>();
        //initial input curves
        private GH_Structure<GH_Curve> inputCurves = new GH_Structure<GH_Curve>();

        //joint skeleton as curves
        private GH_Structure<GH_Curve> skeletonLines = new GH_Structure<GH_Curve>();
        //print slices returned as meshes
        private GH_Structure<GH_Mesh> printSlices_mesh = new GH_Structure<GH_Mesh>();




        private int numX, numY; //number of points per side. 
        private bool planeMode, curveMode;

        private double resolution;
        private double nodeRadius;
        private double gridX;
        private double gridY;
        private double layerHeight;

        

        //IF INPUT IS PLANES
        public NodeBuilder(GH_Structure<GH_Plane> inputPlanes_, double resolution_, double gridX_, double gridY_, double nodeRadius_, double layerHeight_)
        {
            planeMode = true;
            curveMode = false;
            inputPlanes = inputPlanes_;
            resolution = resolution_;
            gridX = gridX_;
            gridY = gridY_;
            nodeRadius = nodeRadius_;
            layerHeight = layerHeight_;

            numX = (int)(gridX / resolution);
            numY = (int)(gridY / resolution);
        }

        //IF INPUT IS CURVES
        public NodeBuilder(GH_Structure<GH_Curve> inputCurves_, double resolution_, double gridX_, double gridY_, double nodeRadius_, double layerHeight_)
        {
            planeMode = false;
            curveMode = true;
            inputCurves = inputCurves_;
            resolution = resolution_;
            gridX = gridX_;
            gridY = gridY_;
            nodeRadius = nodeRadius_;
            layerHeight = layerHeight_;

            numX = (int)(gridX / resolution);
            numY = (int)(gridY / resolution);
        }

        /// <summary>
        /// Generates all geometry for printing. Main functional method.
        /// </summary>
        public void GeneratePrintLayers()
        {
            if (planeMode)
            {
                skeletonLines = BuildStructureAsCurves();

                //iterate through each layer, get intersection, calc point grid, get mesh. 
                //for each joint i
                for (int i = 0; i < Utility.GetMainBranchCount(inputPlanes.Paths); i++) //go through each joint
                {
                    Plane currentPlane = Utility.CastToPlane(inputPlanes.get_DataItem(new GH_Path(i, 0), 0));
                    BuildCurrentPrintLayer(i, currentPlane);
                }
            }

            if (curveMode)
            {
                for (int i = 0; i < inputCurves.PathCount; i++) //go through each joint
                {
                    Plane currentPlane = new Plane();

                    Curve currentCrv = Utility.CastToCurve(inputCurves.get_DataItem(new GH_Path(i), 0));
                    currentCrv.Domain = new Interval(0, 1);
                    currentCrv.PerpendicularFrameAt(0, out currentPlane);

                    BuildCurrentPrintLayer(i, currentPlane);
                }
            }
        }

        private void BuildCurrentPrintLayer(int currentNode, Plane currentPlane)
        {
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
                List<Point3d> CXP = GenerateIntersections(currentNode, currentPlane);
                //check whether there are still intersections
                if (CXP.Count == 0)
                {
                    DebugLog.Add("has no intersections!");
                    hasIntersections = false;
                    break;
                }

                //calc printSlices
                List<Point3d> slicePoints = GenerateSlicePoints(tempGrid, CXP);



                //build mesh for slice
                Mesh m = BuildMeshFromSlicePoints(currentPlane, slicePoints);

                GH_Path pth = new GH_Path(currentNode, currentLayer - 1);
                printSlices_mesh.Append(new GH_Mesh(m), pth);

                currentLayer++;

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
                        if (distToCenter < nodeRadius)
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
            if (planeMode)
            {
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
            }
            if (curveMode)
            {
                foreach (GH_Curve c in inputCurves.get_Branch(jointNumber))
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
    }

}
