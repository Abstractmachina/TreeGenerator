
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TreeGenerator
{
    class StemBuilder
    {
        //INPUT
        private GH_Structure<GH_Curve> inputSkeleton = new GH_Structure<GH_Curve>();
        private double radius;
        private double jointHeight;
        private double jointRadius;
        private double layerHeight; 


        //OUTPUT
        public List<string> DebugLog;


        public StemBuilder(GH_Structure<GH_Curve> inputSkeleton_, double radius_, double jointHeight_, double jointRadius_, double layerHeight_)
        {
            inputSkeleton = inputSkeleton_;
            radius = radius_;
            jointHeight = jointHeight_;
            jointRadius = jointRadius_;
            layerHeight = layerHeight_;
            DebugLog = new List<string>();
        }

        public void GeneratePrintLayers()
        {

        }




    }
}
