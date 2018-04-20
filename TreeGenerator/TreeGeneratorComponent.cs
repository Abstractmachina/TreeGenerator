using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;


namespace TreeGenerator
{
    //test
    public class TreeGeneratorComponent : GH_Component
    {
        public TreeGeneratorComponent()
          : base("TreeGenerator", "TreeG",
              "Generate an agent-based tree",
              "Generative", "Agent-Based")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Seeds", "S", "Points where Agents are seeded", GH_ParamAccess.list, new Point3d(0, 0, 0));
            pManager.AddNumberParameter("Angle", "A", "Angle of Branching.", GH_ParamAccess.item, 1d);
            pManager.AddMeshParameter("Boundary", "B", "Boundary that agents can't cross.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Reset", "R1", "Reset Simulation.", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Run", "R2", "Run Simulation.", GH_ParamAccess.item, false);
            pManager.AddNumberParameter("Manual Animation", "M", "Attach a slide for manual animation", GH_ParamAccess.item, 0.0d);
            pManager.AddNumberParameter("Plane Resolution", "PD", "Resolution of planes that describe the structure.", GH_ParamAccess.item, 10d);

            // If you want to change properties of certain parameters, 
            // you can use the pManager instance to access them by index:
            //pManager[0].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Planes of tree structure.", GH_ParamAccess.tree);
            pManager.AddTextParameter("Debug Log", "D", "log messages for debugging.", GH_ParamAccess.list);
            pManager.AddPointParameter("Agents", "A", "Agent Geometry", GH_ParamAccess.list);
            pManager.AddLineParameter("BoundaryShortestPath", "BP", "Shortest Path to Boundary", GH_ParamAccess.list);
            pManager.AddPointParameter("Fuse Points", "FP", "Points where branches have merged", GH_ParamAccess.list);


            // Sometimes you want to hide a specific parameter from the Rhino preview.
            // You can use the HideParameter() method as a quick way:
            //pManager.HideParameter(0);
        }

        bool reset;
        bool run;

        GH_Structure<GH_Plane> outputPlanes;
        List<string> debugLog = new List<string>();


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double manualAnimDump = 0;
            if (!DA.GetData(1, ref angle)) return;
            if (!DA.GetData(3, ref reset)) return;
            if (!DA.GetData(4, ref run)) return;
            if (DA.GetData(5, ref manualAnimDump)) { };
            if (!DA.GetData(6, ref recordFrequency)) return;
            if (manualAnimDump != 0) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "MANUAL MODE ACTIVE");



            if (reset)
            {
                List<Point3d> tempSeeds = new List<Point3d>();


                if (!DA.GetDataList<Point3d>(0, tempSeeds)) return;
                if (!DA.GetData(2, ref boundary)) return;

                Setup(tempSeeds); // todo somehow does not exit method. setupTriggered shows nothing below this. 

            }

            //add runtime messages for user
            if (angle == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Angle must be > 0.0.");
                return;
            }

            if (run) Update();

            debugLog.Clear();
            debugLog.Add("frame count: " + frameCount);
            debugLog.Add("agents created: " + agentsCreated);
            debugLog.Add("active Agents: " + sim.activeAgents.Count);
            debugLog.Add("Branching triggered: " + branchTriggered);
            debugLog.Add("died childless: " + childlessDeathTrigger);
            debugLog.Add("dead agents: " + deadAgentCount);

            //data container for output
            outputPlanes = sim.geometry.planes;

            List<Point3d> outputAgents = OutputPoints(sim.activeAgents);

            //set data output
            DA.SetDataTree(0, outputPlanes);
            DA.SetDataList(1, debugLog);
            DA.SetDataList(2, outputAgents);
            DA.SetDataList(3, boundaryLineOfSight);
            DA.SetDataList(4, fusePoints);
        }



        //CUSTOM CODE START
        //======================================================================================================================================


        public static int agentCount;
        public static TreeSimulator sim;
        public static double recordFrequency;

        //general simulation variables
        public static int minLifeSpan = 100;
        public static int maxLifeSpan = 300;
        public static int numberOfNewChildren = 3;
        public static Vector3d lightDir = new Vector3d(0, 0, 0.01f);
        public static double angle; //angle of branching //in degrees
        public static double minGrowthSpeed = 0.2;
        public static double maxGrowthSpeed = 0.6;
        public static double fuseAlignThreshold = 4d; //in modelspace units
        public static double fuseThreshold = 0.1; //in modelspace units
        public static double fuseFactor = 0.4;
        public static double FuseImmuneDistance = 2d;

        public static double separationFactor = 0.002f;
        public static double separationMax = 20f;
        public static double separationMin = 0.001f;

        //boundary variables
        public static Mesh boundary;
        public static double collisionTriggerDistance = 0.35;

        //DEBUG MODE
        public static int fuseCount;
        public static int boundaryCollisionTrigger;
        public static List<Line> boundaryLineOfSight = new List<Line>();
        public static List<Point3d> fusePoints = new List<Point3d>();
        public static int frameCount;
        public static int childlessDeathTrigger;
        public static int sameParentsFound;
        public static int agentsCreated;
        public static int branchTriggered;
        public static int deadAgentCount;


        public void Setup(List<Point3d> seeds) //pass in any geometry/data that is to be used in the script
        {
            //DEBUG RESET
            ResetDebugValues();
            
            sim = new TreeSimulator();
            sim.PlantSeeds(seeds);
        }

        public void Update()
        {
            sim.Update();
        }




        // ===============================================================================================
        //===================  CLASS : TREE SIMULATOR  ===================================================
        // ===============================================================================================
        //contains all movement functionality (forces, animation etc)

        public class TreeSimulator
        {
            //====================================================================  VARIABLES

            public Geometry geometry { get; protected set; }
            public List<Agent> activeAgents;
            public string msg = "";

            public Random r;

            //====================================================================  CONSTRUCTOR
            public TreeSimulator()
            {
                geometry = new Geometry();
                activeAgents = new List<Agent>();
                r = new Random();

                msg = "registered";
            }

            //===================================================================   METHODS

            public void PlantSeeds(List<Point3d> seedPts)
            {
                Vector3d rootVec = new Vector3d(0, 0, 0.5f);
                foreach (Point3d p in seedPts)
                {
                    Agent a = new Agent((Vector3d)p, rootVec, r.Next(minLifeSpan, maxLifeSpan));
                    a.parentId = 0;
                    sim.activeAgents.Add(a);
                    sim.geometry.SavePlane(new Plane((Point3d)a.loc, a.vel), a.id);
                }
            }


            public void Update()
            {
                foreach (Agent a in activeAgents)
                {
                    a.Update();
                }
                UpdateAllAgents();
                geometry.Record(activeAgents);

                if (activeAgents.Count != 0) frameCount++;
            }


            private void UpdateAllAgents()
            {
                CheckforDeadAgents();
                ProcessGlobalRepulsion();
                ProcessGlobalFusion();// TODO fix fusion. does not work at all.
                boundaryLineOfSight.Clear();
                foreach (Agent a in activeAgents)
                {
                    a.Update();
                    AddLightSource(a, lightDir);
                }
            }


            private void AddLightSource(Agent a, Vector3d dir)
            {
                a.accel += dir;
            }


            private void ProcessGlobalRepulsion()
            {
                for (int i = 0; i < activeAgents.Count; i++)
                {
                    for (int j = 0; j < activeAgents.Count; j++)
                    {
                        if (i != j) activeAgents[i].Separate(activeAgents[j].loc);
                    }
                }
            }


            //private void ProcessGlobalFusion()
            //{
            //    foreach (Agent a in activeAgents)
            //    {
            //        List<int> sameParents = FindSameParents(a); //list of branches that have the same parent
            //        sameParentsFound = sameParents.Count;

            //        Vector3d averageVel = new Vector3d(0, 0, 0); //storage for average alignment vector
            //        int count = 0; //counts number of neighbors

            //        for (int i = 0; i < geometry.planes.PathCount; i++)
            //        {
            //            for (int j = 0; j < geometry.planes.get_Branch(i).Count; j++)
            //            { //nested for loop to check every item in data tree against Agent a

            //                bool hasSameParents = false;
            //                foreach (int k in sameParents)
            //                {
            //                    if (i == a.id || i == k)
            //                    {
            //                        hasSameParents = true;
            //                        break;
            //                    }
            //                }
            //                ;
            //                Vector3d vecToParentNode = (Vector3d)geometry.planes.get_DataItem(new GH_Path(a.id), 0).Value.Origin - a.loc; //calc vector to this agent's parent node
            //                if (vecToParentNode.Length > FuseImmuneDistance && !hasSameParents) //if agent is far enough from parent and vertex does not have the same parent
            //                {
            //                    Vector3d vecToBranchVertex = (Vector3d)geometry.planes.get_DataItem(new GH_Path(i), j).Value.Origin - a.loc; //calculate vector to each branch vertex

            //                    //for calculating alignment vel
            //                    averageVel = CalculateAverageVector(ref averageVel, vecToBranchVertex, ref count);


            //                    //for calculating fusion
            //                    if (vecToBranchVertex.Length < fuseThreshold)
            //                    {
            //                        a.DieChildLess();
            //                        fuseCount++;
            //                    }
            //                }
            //            }
            //        }
            //        //CalculateAlignment(a, count, averageVel);
            //    }
            //}

            private void ProcessGlobalFusion()
            {
                foreach (Agent a in activeAgents)
                {

                    List<int> sameParents = FindSameParents(a);
                    //find all branches with the same parent
                    //for (int j =0; j < activeAgents.Count; j++)
                    //{
                    //    int thisId = a.parentId;
                    //    int otherId = activeAgents[j].parentId;

                    //    if (thisId == otherId) sameParents.Add(j);
                    //}


                    for (int i = 0; i < geometry.planes.PathCount; i++)
                    {
                        foreach (GH_Plane plane in geometry.planes.get_Branch(i))
                        {
                            bool isFuseable = true;
                            foreach (int id in sameParents)
                            {
                                if (id == i)
                                {
                                    isFuseable = false;
                                    break;
                                }
                            }
                            double distToParent = ((Vector3d)geometry.planes.get_DataItem(new GH_Path(a.id), 0).Value.Origin - a.loc).Length;
                            if (i != a.id && distToParent > FuseImmuneDistance)
                            {
                                Plane p = new Plane();
                                plane.CastTo<Plane>(out p);
                                double dist = ((Vector3d)p.Origin - a.loc).Length;
                                if (dist < fuseThreshold)
                                {
                                    a.DieChildLess();
                                    Point3d fusePoint = new Point3d(a.loc);
                                    fusePoints.Add(fusePoint);
                                }
                            }
                            
                        }
                    }
                }
            }


            private List<int> FindSameParents(Agent a)
            {
                List<int> tempList = new List<int>();
                for (int i = 0; i < geometry.planes.PathCount; i++)
                {
                    double nodeDist = ((Vector3d)geometry.planes.get_DataItem(new GH_Path(i), 0).Value.Origin - (Vector3d)geometry.planes.get_DataItem(new GH_Path(a.id), 0).Value.Origin).Length;
                    if (nodeDist < 0.01) tempList.Add(i);
                }
                return tempList;
            }


            private Vector3d CalculateAverageVector(ref Vector3d averageVel, Vector3d distanceVector, ref int count)
            {
                Vector3d avgVec = new Vector3d();
                double dist = distanceVector.Length;
                if (dist < fuseAlignThreshold) //if branchvertex is close enough
                {
                    avgVec += distanceVector;
                    count++;
                }
                return avgVec;
            }


            private void CalculateAlignment(Agent a, Vector3d distanceVector)
            {
                Vector3d difference = distanceVector - a.vel;
                difference.Unitize();
                a.vel += difference * fuseFactor;
            }


            private void CalculateAlignment(Agent a, int count, Vector3d averageVel)
            {
                if (count != 0)
                {
                    Vector3d difference = averageVel - a.vel;
                    difference.Unitize();
                    a.vel += difference * fuseFactor;
                }
            }


            //Behavior on death
            private void CheckforDeadAgents()
            {
                if (FoundDeadAgents()) PurgeAgents();
            }

            private void PurgeAgents()
            {
                for (int i = 0; i < activeAgents.Count; i++)
                {
                    Agent a = activeAgents[i];
                    if (a.isDead)
                    {
                        geometry.SavePlane(new Plane((Point3d)a.loc, a.vel), a.id);
                        deadAgentCount++;
                        if (a.childVecs.Count != 0) //if its possible to have children, i.e. it died of old age
                        {
                            foreach (Vector3d childVec in a.childVecs)
                            {
                                Agent child = new Agent(a.loc, childVec, r.Next(minLifeSpan, maxLifeSpan));
                                child.parentId = a.id;
                                geometry.SavePlane(new Plane((Point3d)child.loc, child.vel), child.id);
                                activeAgents.Add(child);
                            }
                        }
                        activeAgents.Remove(a);
                    }
                }
            }

            private bool FoundDeadAgents()
            {
                bool tempCheck = false;
                foreach (Agent a in activeAgents)
                {
                    if (a.isDead == true)
                    {
                        tempCheck = true;
                        break;
                    }
                }
                return tempCheck;
            }

           
            //END OF CLASS
        }


        // ===============================================================================================
        //===================  CLASS : Geometry ==========================================================
        // ===============================================================================================
        //collects geometry and provides geometry-recording functionality

        public class Geometry
        {
            //============= VARIABLES ===================
            public GH_Structure<GH_Plane> planes { get; protected set; }

            // TODO switch to time-based units
            
            private int currentRecordFrame;



            //============= CONSTRUCTOR ===================

            public Geometry()
            {
                planes = new GH_Structure<GH_Plane>();
            }


            //==============  METHODS ===================

            /// <summary>
            /// Record geometry per specified frequency.
            /// </summary>
            /// <param name="planes">test.</param>
            /// <param name="branchIndex"></param>
            /// <returns></returns>
            public void Record(List<Plane> planes, int branchIndex)
            {
                if (CheckRecInterval())
                {
                    SavePlanes(planes, branchIndex);
                }
            }
            public void Record(List<Agent> agents)
            {
                if (CheckRecInterval())
                {
                    foreach (Agent a in agents)
                    {
                        Plane p = new Plane((Point3d)a.loc, a.vel);
                        SavePlane(p, a.id);
                    }
                }
            }

            private bool CheckRecInterval()
            {
                currentRecordFrame++;
                if (currentRecordFrame >= recordFrequency)
                {
                    currentRecordFrame = 0;
                    return true;
                }
                else return false;
            }

            /// <summary>
            /// Save a single plane to data tree.
            /// </summary>
            /// <param name="plane"></param>
            /// <param name="branchIndex"></param>
            public void SavePlane(Plane plane, int branchIndex) //save point of current activeAgents to datatree
            {

                GH_Path path = new GH_Path(branchIndex);
                planes.Append(new GH_Plane(plane), path);
            }

            /// <summary>
            /// Save a List of planes to data tree.
            /// </summary>
            /// <param name="planes"></param>
            /// <param name="branchIndex"></param>
            private void SavePlanes(List<Plane> planes, int branchIndex)
            {
                foreach (Plane p in planes)
                {
                    SavePlane(p, branchIndex);
                }
            }

            //SETTERS GETTERS
        }





        // ===============================================================================================
        //===================  CLASS : AGENT =============================================================
        // ===============================================================================================
        //single agent behaviour
        public class Agent
        {

            //============= VARIABLES ===================

            
            public Vector3d loc;
            public Vector3d vel;
            public Vector3d accel;

            public List<Vector3d> childVecs;


            public int id;
            public int parentId;
            public int age;
            public int lifeSpan;
            public int deathRate = 1;
            public bool isDead = false;
            public bool canBranch = true;

            private double growthSpeed;






            //============= CONSTRUCTOR ===================

            public Agent(Vector3d loc_, Vector3d initVec_, int lifeSpan_)
            {

                loc = loc_;
                vel = initVec_;
                id = agentCount;
                agentCount++;
                lifeSpan = lifeSpan_;
                childVecs = new List<Vector3d>();
                growthSpeed = 0.6;
                agentsCreated++;
            }


            //==============  METHODS ===================
            public void Update()
            {
                if (age >= lifeSpan) Die();
                if (!isDead) Move();
            }


            private void Move()
            {
                ConstrainGrowthSpeed();
                CheckBoundaryCollision();
                age += deathRate;
                vel += accel;
                loc += vel;
                accel *= 0;
            }


            private void Die() //die while creating children, if only switching isDead to true, no children will be created
            {
                isDead = true;
                if (canBranch)
                {
                    double angleRad = DegToRad(angle);
                    for (int i = 0; i < numberOfNewChildren; i++)
                    {
                        Vector3d childVec = vel; //initVector same as dying parent
                        childVec.Transform(Rhino.Geometry.Transform.Rotation(angleRad, new Vector3d(0, 1, 0), (Point3d)loc)); //rotate in elevation
                        childVec.Transform(Rhino.Geometry.Transform.Rotation(DegToRad(120) * i, vel, (Point3d)loc)); //rotate in normal randomly

                        childVecs.Add(childVec); //add to list of childs
                    }
                    canBranch = false; //after branching once, it cannot branch again.
                }
            }

            public void DieChildLess()
            {
                isDead = true;
                canBranch = false;
            }


            private void ConstrainGrowthSpeed()
            {
                if (vel.Length > growthSpeed)
                {
                    vel.Unitize();
                    vel *= growthSpeed;
                }
            }


            private void CheckBoundaryCollision()
            {

                Vector3d closestPoint = (Vector3d)boundary.ClosestPoint((Point3d)loc);
                double dist = (closestPoint - loc).Length;
                Line l = new Line((Point3d)closestPoint, (Point3d)loc);
                boundaryLineOfSight.Add(l);

                if (dist <= collisionTriggerDistance)
                {
                    canBranch = false;
                    isDead = true;
                    boundaryCollisionTrigger++;

                    if (dist != 0)
                    {
                        loc = (Vector3d)closestPoint;
                    }
                }
            }


            public void Separate(Vector3d other)
            {
                Vector3d vec = other - loc;
                double l = vec.Length;
                if (l < separationMax && l > separationMin)
                {
                    double fallOff = ReMap(l, separationMin, separationMax, 1, 0);
                    vel += vec * -1 * separationFactor * fallOff;
                }
            }


            //GETTERS SETTERS
            public List<Vector3d> getChildVectors() { return childVecs; }

            public void SetID(int ID) { id = ID; }

            public void SetAge(int age_) { age = age_; }
            public int GetAge() { return age; }
        }




        // ===============================================================================================
        //======================  UTILITY   ==============================================================
        // ===============================================================================================
        public List<Point3d> OutputPoints(List<Agent> agents)
        {
            List<Point3d> Pts = new List<Point3d>();
            foreach (Agent a in agents)
            {
                Point3d pt = new Point3d(a.loc);
                Pts.Add(pt);
            }
            return Pts;
        }

        public static double ReMap(double value, double from1, double to1, double from2, double to2)
        {
            return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
        }

        public static double DegToRad(double dAngle)
        {
            double rAngle = dAngle * Math.PI / 180;
            return rAngle;

        }

        public static void ResetDebugValues()
        {
            frameCount = 0;
            agentCount = 0;
            agentsCreated = 0;
            childlessDeathTrigger = 0;
            branchTriggered = 0;
            deadAgentCount = 0;
            boundaryLineOfSight.Clear();
            fusePoints.Clear();
        }

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



        //======================================================================================================================================
        //END OF CUSTOM CODE








        /// <summary>
        /// The Exposure property controls where in the panel a component icon 
        /// will appear. There are seven possible locations (primary to septenary), 
        /// each of which can be combined with the GH_Exposure.obscure flag, which 
        /// ensures the component will only be visible on panel dropdowns.
        /// </summary>
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("e885d2a8-d128-4ead-8a05-559e80c74fdb"); }
        }
    }
}
