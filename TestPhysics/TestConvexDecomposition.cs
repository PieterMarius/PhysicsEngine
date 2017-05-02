using ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace TestPhysics
{
    public class TestConvexDecomposition
    {
        #region Fields

        public struct ShapePiece
        {
            public PhysicsEngineMathUtility.Vector3 IntersectionPoint; //Max => 8 index points
            public VertexAdjacency[] Triangle;
        };

        public List<PhysicsEngineMathUtility.Vector3> spherePoint = new List<PhysicsEngineMathUtility.Vector3>();

        #endregion

        #region Public Methods

        public void TestNonConvexDec()
        {
            //Utility.ObjImporter importer = new ObjImporter();
            //Utility.ObjImporter.meshStruct mesh = importer.ImportFile("chair.obj");

            IGeometry obj = LoadObject.GetObjectGeometry(null, "bunny.obj", 1.0f, ObjectGeometryType.NonConvexBody);
            //Geometry obj = LoadObject.GetObjectGeometry(null, "chair.obj", 1.0f);

            Stopwatch stopwatch = new Stopwatch();

            spherePoint = new List<PhysicsEngineMathUtility.Vector3>();

            //Suddivido il volume in parti
            int nSphere = 20;

            var test = obj.VertexPosition.Where(x => x.Adjacency.Count < 6).ToList();

            List<ShapePiece> basePoint = new List<ShapePiece>();


            //Provo a verificare di farlo per ogni lato del cubo
            double sphereGap = 2.1 / nSphere;

            //Direction 1
            PhysicsEngineMathUtility.Vector3 direction1 = new PhysicsEngineMathUtility.Vector3(0.0, 0.0, -1.0);
            PhysicsEngineMathUtility.Vector3 direction2 = new PhysicsEngineMathUtility.Vector3(0.0, 0.0, 1.0);
            PhysicsEngineMathUtility.Vector3 direction3 = new PhysicsEngineMathUtility.Vector3(0.0, -1.0, 0.0);
            PhysicsEngineMathUtility.Vector3 direction4 = new PhysicsEngineMathUtility.Vector3(0.0, 1.0, 0.0);
            PhysicsEngineMathUtility.Vector3 direction5 = new PhysicsEngineMathUtility.Vector3(-1.0, 0.0, 0.0);
            PhysicsEngineMathUtility.Vector3 direction6 = new PhysicsEngineMathUtility.Vector3(1.0, 0.0, 0.0);

            PhysicsEngineMathUtility.Vector3 startPoint1 = new PhysicsEngineMathUtility.Vector3(-1.05, -1.05, 0.0);
            PhysicsEngineMathUtility.Vector3 startPoint2 = new PhysicsEngineMathUtility.Vector3(-1.05, 0.0, -1.05);
            PhysicsEngineMathUtility.Vector3 startPoint3 = new PhysicsEngineMathUtility.Vector3(0.0, -1.05, -1.05);


            stopwatch.Reset();
            stopwatch.Start();
            //Parallel.For(0,
            //    nSphere,
            //    new ParallelOptions { MaxDegreeOfParallelism = 4 },
            //    i =>
            for (int i = 0; i < nSphere; i++)
            {
                for (int k = 0; k < nSphere; k++)
                {
                    PhysicsEngineMathUtility.Vector3 point1 = startPoint1 + new PhysicsEngineMathUtility.Vector3(i * sphereGap, k * sphereGap, 0.0);
                    PhysicsEngineMathUtility.Vector3 point2 = startPoint2 + new PhysicsEngineMathUtility.Vector3(i * sphereGap, 0.0, k * sphereGap);
                    PhysicsEngineMathUtility.Vector3 point3 = startPoint3 + new PhysicsEngineMathUtility.Vector3(0.0, i * sphereGap, k * sphereGap);



                    for (int j = 0; j < obj.Triangle.Length; j++)
                    {
                        PhysicsEngineMathUtility.Vector3 vertexA = obj.VertexPosition[obj.Triangle[j][0]].Vertex;
                        PhysicsEngineMathUtility.Vector3 vertexB = obj.VertexPosition[obj.Triangle[j][1]].Vertex;
                        PhysicsEngineMathUtility.Vector3 vertexC = obj.VertexPosition[obj.Triangle[j][2]].Vertex;


                        PhysicsEngineMathUtility.Vector3? intersection = PhysicsEngineMathUtility.GeometryUtilities.RayTriangleIntersection(
                                vertexA,
                                vertexB,
                                vertexC,
                                point1,
                                direction1,
                                true);

                        if (intersection.HasValue)
                        {
                            spherePoint.Add(intersection.Value);
                            basePoint.Add(new ShapePiece() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[obj.Triangle[j][0]], obj.VertexPosition[obj.Triangle[j][1]], obj.VertexPosition[obj.Triangle[j][2]] } });
                            continue;
                        }

                        intersection = PhysicsEngineMathUtility.GeometryUtilities.RayTriangleIntersection(
                            vertexA,
                            vertexB,
                            vertexC,
                            point2,
                            direction3,
                            true);

                        if (intersection.HasValue)
                        {
                            spherePoint.Add(intersection.Value);
                            basePoint.Add(new ShapePiece() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[obj.Triangle[j][0]], obj.VertexPosition[obj.Triangle[j][1]], obj.VertexPosition[obj.Triangle[j][2]] } });
                            continue;
                        }

                        intersection = PhysicsEngineMathUtility.GeometryUtilities.RayTriangleIntersection(
                            vertexA,
                            vertexB,
                            vertexC,
                            point3,
                            direction5,
                            true);

                        if (intersection.HasValue)
                        {
                            spherePoint.Add(intersection.Value);
                            basePoint.Add(new ShapePiece() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[obj.Triangle[j][0]], obj.VertexPosition[obj.Triangle[j][1]], obj.VertexPosition[obj.Triangle[j][2]] } });
                            continue;
                        }

                    }


                }
            }

            double ray = sphereGap / 2.0;
            List<int>[] index = new List<int>[spherePoint.Count];
            int totalCount = 0;
            for (int i = 0; i < spherePoint.Count; i++)
            {
                PhysicsEngineMathUtility.Vector3 point = spherePoint[i];
                index[i] = new List<int>();
                for (int j = 0; j < obj.VertexPosition.Length; j++)
                {
                    if (PhysicsEngineMathUtility.Vector3.Length(point - obj.VertexPosition[j].Vertex) <= ray)
                        index[i].Add(j);
                }
                totalCount += index[i].Count;
            }

            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);
        }

        #endregion
    }
}
