using PhysicsEngineMathUtility;
using ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;

namespace TestPhysics
{
    public class NonConvexDecomposition
    {
        #region Fields

        public struct NonConvexPoint
        {
            public Vector3 IntersectionPoint; //Max => 8 index points
            public VertexAdjacency[] Triangle;
        };

        public List<NonConvexPoint> basePoint = new List<NonConvexPoint>();

        #endregion

        #region Public Methods

        public void Decompose(double diameter)
        {
            //Utility.ObjImporter importer = new ObjImporter();
            //Utility.ObjImporter.meshStruct mesh = importer.ImportFile("chair.obj");

            IGeometry obj = null;
            //IGeometry obj = LoadObject.GetObjectGeometry(null, "teapot.obj", 1.0f, ObjectGeometryType.NonConvexBody);
            //Geometry obj = LoadObject.GetObjectGeometry(null, "chair.obj", 1.0f);

            Stopwatch stopwatch = new Stopwatch();

            //Suddivido il volume in parti
            int nSphere = Convert.ToInt32(2.1 / diameter);

            var test = obj.VertexPosition.Where(x => x.Adjacency.Count < 6).ToList();

            //Provo a verificare di farlo per ogni lato del cubo
            double sphereGap = 2.1 / nSphere;

            //Direction 1
            Vector3 direction1 = new Vector3(0.0, 0.0, -1.0);
            Vector3 direction2 = new Vector3(0.0, 0.0, 1.0);
            Vector3 direction3 = new Vector3(0.0, -1.0, 0.0);
            Vector3 direction4 = new Vector3(0.0, 1.0, 0.0);
            Vector3 direction5 = new Vector3(-1.0, 0.0, 0.0);
            Vector3 direction6 = new Vector3(1.0, 0.0, 0.0);

            Vector3 startPoint1 = new Vector3(-1.05, -1.05, 0.0);
            Vector3 startPoint2 = new Vector3(-1.05, 0.0, -1.05);
            Vector3 startPoint3 = new Vector3(0.0, -1.05, -1.05);
            
            stopwatch.Reset();
            stopwatch.Start();

            var sync = new object();

            Parallel.For(0,
                nSphere,
                new ParallelOptions { MaxDegreeOfParallelism = 6 },
                i =>
                {
                    //for (int i = 0; i < nSphere; i++)
                    //{
                    double va = i * sphereGap;
                    for (int k = 0; k < nSphere; k++)
                    {                        
                        double vb = k * sphereGap;

                        Vector3 point1 = startPoint1 + new Vector3(va, vb, 0.0);
                        Vector3 point2 = startPoint2 + new Vector3(va, 0.0, vb);
                        Vector3 point3 = startPoint3 + new Vector3(0.0, va, vb);

                        foreach(var triangle in obj.Triangle)
                        {
                            Vector3 vertexA = obj.VertexPosition[triangle[0]].Vertex;
                            Vector3 vertexB = obj.VertexPosition[triangle[1]].Vertex;
                            Vector3 vertexC = obj.VertexPosition[triangle[2]].Vertex;
                            
                            Vector3? intersection = GeometryUtilities.RayTriangleIntersection(
                                    vertexA,
                                    vertexB,
                                    vertexC,
                                    point1,
                                    direction1,
                                    true);

                            if (intersection.HasValue)
                            {
                                lock (sync)
                                {
                                    basePoint.Add(new NonConvexPoint() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[triangle[0]], obj.VertexPosition[triangle[1]], obj.VertexPosition[triangle[2]] } });
                                }
                                continue;
                            }

                            intersection = GeometryUtilities.RayTriangleIntersection(
                                vertexA,
                                vertexB,
                                vertexC,
                                point2,
                                direction3,
                                true);

                            if (intersection.HasValue)
                            {
                                lock (sync)
                                {
                                    basePoint.Add(new NonConvexPoint() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[triangle[0]], obj.VertexPosition[triangle[1]], obj.VertexPosition[triangle[2]] } });
                                }
                                continue;
                            }

                            intersection = GeometryUtilities.RayTriangleIntersection(
                                vertexA,
                                vertexB,
                                vertexC,
                                point3,
                                direction5,
                                true);

                            if (intersection.HasValue)
                            {
                                lock (sync)
                                {
                                    basePoint.Add(new NonConvexPoint() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[triangle[0]], obj.VertexPosition[triangle[1]], obj.VertexPosition[triangle[2]] } });
                                }
                                continue;
                            }

                        }
                    }
                });

            //double ray = sphereGap / 2.0;
            //List<int>[] index = new List<int>[spherePoint.Count];
            //int totalCount = 0;
            //for (int i = 0; i < spherePoint.Count; i++)
            //{
            //    PhysicsEngineMathUtility.Vector3 point = spherePoint[i];
            //    index[i] = new List<int>();
            //    for (int j = 0; j < obj.VertexPosition.Length; j++)
            //    {
            //        if (PhysicsEngineMathUtility.Vector3.Length(point - obj.VertexPosition[j].Vertex) <= ray)
            //            index[i].Add(j);
            //    }
            //    totalCount += index[i].Count;
            //}

            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);
        }

        #endregion
    }
}
