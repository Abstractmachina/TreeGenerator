using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace TreeGenerator
{
    public static class Utility
    {

        #region GH_Structures Utility


        /// <summary>
        /// Returns number of branches in the first dimension of a data tree.
        /// </summary>
        /// <param name="paths">Data paths of a given tree.</param>
        /// <returns></returns>
        public static int GetMainBranchCount(IList<GH_Path> paths)
        {
            HashSet<int> mainBranches = new HashSet<int>();
            foreach (GH_Path path in paths)
            {
                mainBranches.Add(path.Indices[0]);
            }
            return mainBranches.Count;
        }

        /// <summary>
        /// Returns number of branches in the second dimension of a given main branch of a tree, so { ofBranch ; x }.
        /// </summary>
        /// <param name="paths"></param>
        /// <param name="ofBranch"></param>
        /// <returns></returns>
        public static int GetSecondaryBranchCount(IList<GH_Path> paths, int ofBranch)
        {
            int branchCount = 0;
            foreach (GH_Path path in paths)
            {
                if (path[0] == ofBranch)
                {
                    branchCount++;
                }
            }
            return branchCount;
        }
        /// <summary>
        /// Returns number of branches in the third dimension where the first and second dimensions are specified, so { ofBranch0 ; ofBranch1; x }.
        /// </summary>
        /// <param name="paths"></param>
        /// <param name="OfBranch0"></param>
        /// <param name="ofBranch1"></param>
        /// <returns></returns>
        public static int GetTertiaryBranchCount(IList<GH_Path> paths, int OfBranch0, int ofBranch1)
        {
            int branchCount = 0;
            foreach (GH_Path path in paths)
            {
                if (path[0] == OfBranch0 && path[1] == ofBranch1)
                {
                    branchCount++;
                }
            }
            return branchCount;
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="paths"></param>
        /// <returns></returns>
        public static HashSet<int> GetMainBranches(IList<GH_Path> paths)
        {
            HashSet<int> mainBranches = new HashSet<int>();
            foreach (GH_Path path in paths)
            {
                mainBranches.Add(path.Indices[0]);
            }
            return mainBranches;
        }

        public static HashSet<int> GetSecondaryBranches(IList<GH_Path> paths, int ofBranch)
        {
            HashSet<int> branches2d = new HashSet<int>();
            foreach (GH_Path path in paths)
            {
                if (path.Indices[0] == ofBranch) branches2d.Add(path.Indices[1]);
            }
            return branches2d;
        }


        #endregion

        #region Cast Utility

        //----------------------------------------------- TO RHINO TYPES
        public static Plane CastToPlane(GH_Plane p)
        {
            Plane pl = new Plane();
            p.CastTo<Plane>(out pl);
            return pl;
        }

        public static Point3d CastToPoint3d(GH_Point p)
        {
            Point3d p3d = new Point3d();
            p.CastTo<Point3d>(out p3d);
            return p3d;
        }


        //--------------------------------------------- FROM RHINO TYPES

        public static GH_Curve CastFromPLine(Polyline crv)
        {
            GH_Curve outcv = new GH_Curve();
            outcv.CastFrom(crv);
            return outcv;
        }

        #endregion



        #region Math Utility
        /// <summary>
        /// Remaps a domain.
        /// </summary>
        /// <param name="value"></param>
        /// <param name="from1"></param>
        /// <param name="to1"></param>
        /// <param name="from2"></param>
        /// <param name="to2"></param>
        /// <returns></returns>
        public static double ReMap(double value, double from1, double to1, double from2, double to2)
        {
            return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="dAngle"></param>
        /// <returns></returns>
        public static double DegToRad(double dAngle)
        {
            double rAngle = dAngle * Math.PI / 180;
            return rAngle;

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public static double RandomDouble(double from, double to)
        {
            int accuracy = 1000;
            Random r = new Random();
            int fromTemp = (int)from * accuracy;
            int toTemp = (int)to * accuracy;
            double random = r.Next(fromTemp, toTemp);
            random *= 1 / accuracy;
            return random;

        }
        #endregion
    }
}