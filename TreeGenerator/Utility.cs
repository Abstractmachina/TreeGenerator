using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;

namespace TreeGenerator
{
    public static class Utility
    {
        public static int GetMainBranchCount(IList<GH_Path> paths)
        {
            HashSet<int> mainBranches = new HashSet<int>();
            foreach (GH_Path path in paths)
            {
                mainBranches.Add(path.Indices[0]);
            }
            return mainBranches.Count;
        }

        public static int GetSecondaryBranchCount(IList<GH_Path> paths, int OfBranch)
        {
            int branchCount = 0;
            foreach (GH_Path path in paths)
            {
                if (path[0] == OfBranch)
                {
                    branchCount++;
                }
            }
            return branchCount;
        }
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
    }
}