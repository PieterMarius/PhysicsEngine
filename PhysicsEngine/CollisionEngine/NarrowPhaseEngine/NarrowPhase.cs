/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using SharpEngineMathUtility;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class NarrowPhase : INarrowPhase
    {
        #region Private Fields

        private const double normalTolerance = 1E-15;
                
        private readonly CollisionEngineParameters parameters;
        private readonly AABBBroadPhase broadPhaseCollisionEngine;
        private readonly ConvexBodyNarrowPhase convexBodyNarrowPhase;
        private readonly K_Means.KMeans kMeansEngine;

        #endregion

        #region Constructor

        public NarrowPhase(CollisionEngineParameters parameters)
        {
            this.parameters = parameters;

            convexBodyNarrowPhase = new ConvexBodyNarrowPhase(parameters);
            broadPhaseCollisionEngine = new AABBBroadPhase(parameters);
            kMeansEngine = new K_Means.KMeans();
        }

        #endregion

        #region Public Methods

        public List<CollisionPointStructure> Execute(
            IShape[] shapes,
            List<CollisionPair> collisionPairs,
            double collisionDistance)
        {
            var result = new List<CollisionPointStructure>();

            foreach (var pair in collisionPairs)
            {
                CollisionPointStructure collisionPointStruct = ExecuteNarrowPhase(
                        shapes[pair.objectIndexA],
                        shapes[pair.objectIndexB],
                        collisionDistance);
               
                if(collisionPointStruct != null)
                    result.Add(collisionPointStruct);
            }

            return result;
        }

        public CollisionPointStructure Execute(
            IShape shapeA,
            IShape shapeB,
            double collisionDistance)
        {
            CollisionPointStructure collisionPointStruct = ExecuteNarrowPhase(
                            shapeA,
                            shapeB,
                            collisionDistance);

            return collisionPointStruct;
        }

        #endregion

        #region Private Methods

        private CollisionPointStructure ExecuteNarrowPhase(
            IShape A,
            IShape B,
            double collisionDistance)
        {
            List<CollisionPointStructure> collisionPointStructure = GetCollisionPointStructure(A, B, collisionDistance);

            if (collisionPointStructure.Count > 1)
            {
                List<CollisionPointBaseStructure> baseStructure = new List<CollisionPointBaseStructure>();

                foreach (CollisionPointStructure cps in collisionPointStructure)
                {
                    if (cps != null)
                        baseStructure.AddRange(cps.CollisionPointBase);
                }

                if (A is ConcaveShape ||
                    B is ConcaveShape)
                {
                    var concaveCollisionPoints = ExtractConcaveShapeCollisionPoint(A, B, baseStructure).ToArray();
                    collisionPointStructure[0].SetBaseCollisionPoint(concaveCollisionPoints);

                }
                else
                {
                    collisionPointStructure[0].SetBaseCollisionPoint(baseStructure.ToArray());
                }
            }

            if (collisionPointStructure.Count > 0)
                return collisionPointStructure[0];

            return null;
        }

        private List<CollisionPointBaseStructure> ExtractConcaveShapeCollisionPoint(
            IShape A,
            IShape B,
            List<CollisionPointBaseStructure> collisionPointBase)
        {
            var collisionPointWithIntersection = collisionPointBase.Where(x => x.CollisionPoint.Intersection).ToArray();

            CollisionPointBaseStructure[] collisionPointBS = collisionPointBase.ToArray();

            if (collisionPointWithIntersection.Length > 0)
                collisionPointBS = collisionPointWithIntersection;
           
            var executeKMeans = kMeansEngine.Execute(collisionPointBS, parameters.ManifoldPointNumber);
                        
            List<CollisionPointBaseStructure> result = new List<CollisionPointBaseStructure>();

            for (int i = 0; i < parameters.ManifoldPointNumber; i++)
            {
                if (executeKMeans[i].Points.Count > 0)
                    result.Add((CollisionPointBaseStructure)executeKMeans[i].Points.Aggregate((i1, i2) => i1.Item2 < i2.Item2 ? i1 : i2).Item1);
            }

            return result;
        }
        
        private List<CollisionPointStructure> GetCollisionPointStructure(
            IShape A,
            IShape B,
            double collisionDistance)
        {
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            ISoftShape softShapeA = A as ISoftShape;
            ISoftShape softShapeB = B as ISoftShape;

            if (softShapeA == null &&
                softShapeB == null)
            {
                return RigidBodyCollisionStep(A, B, collisionDistance);
            }
            else if (softShapeA != null &&
                     softShapeB != null)
            {
                //Soft Body Collision Detection
                collisionPointStructure.AddRange(SoftBodyCollisionStep(softShapeA, softShapeB));
            }
            else if (softShapeB != null &&
                     softShapeA == null)
            {
                collisionPointStructure.AddRange(Rigid_SoftBodyCollisionDetection(A, softShapeB));
            }
            else if (softShapeA != null &&
                     softShapeB == null)
            {
                collisionPointStructure.AddRange(Rigid_SoftBodyCollisionDetection(B, softShapeA));
            }

            //Self collision detection
            //if (softShapeB != null)
            //{
            //	List<CollisionPointBaseStructure> baseCollisionList = new List<CollisionPointBaseStructure>();
            //	baseCollisionList.AddRange(softBodyCollisionDetection.SelfSoftBodyCollisionDetect(softShapeA, CollisionDistance));
            //}

            ////Self collision detection
            //if (softShapeB != null)
            //{
            //	List<CollisionPointBaseStructure> baseCollisionList = new List<CollisionPointBaseStructure>();
            //	baseCollisionList.AddRange(softBodyCollisionDetection.SelfSoftBodyCollisionDetect(softShapeB, CollisionDistance));
            //}

            return collisionPointStructure;
        }

        /// <summary>
		/// Rigid Body Collision Detection for CompoundShape and ConvexShape
		/// </summary>
		/// <param name="A"></param>
		/// <param name="B"></param>
		/// <returns></returns>
		private List<CollisionPointStructure> RigidBodyCollisionStep(
            IShape A,
            IShape B,
            double collisionDistance)
        {
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            IGeometry[] geometryA = ShapeDefinition.Helper.GetGeometry(A);
            IGeometry[] geometryB = ShapeDefinition.Helper.GetGeometry(B);
                        
            List<CollisionPair> collisionPair = CheckGeometryAABB(
                geometryA,
                geometryB);
                        
            if (collisionPair.Count > 0)
            {
                var verticesA = Helper.GetVertexPosition(A);
                var verticesB = Helper.GetVertexPosition(B);

                VertexProperties[] vertexObjA = null;
                VertexProperties[] vertexObjB = null;

                if (A is ConvexShape)
                    vertexObjA = Helper.SetVertexPosition(geometryA[0], verticesA);

                if (B is ConvexShape)
                    vertexObjB = Helper.SetVertexPosition(geometryB[0], verticesB);

                foreach (var collidingPair in collisionPair)
                {
                    if (!(A is ConvexShape))
                        vertexObjA = Helper.SetVertexPosition(geometryA[collidingPair.objectIndexA], verticesA);
                    if (!(B is ConvexShape))
                        vertexObjB = Helper.SetVertexPosition(geometryB[collidingPair.objectIndexB], verticesB);

                    CollisionPointStructure collision = convexBodyNarrowPhase.Execute(vertexObjA, vertexObjB, A.ID, B.ID, collisionDistance);

                    if (collision != null)
                        collisionPointStructure.Add(collision);
                }
            }

            return collisionPointStructure;
        }

        private List<CollisionPair> CheckGeometryAABB(
            IGeometry[] geometryA,
            IGeometry[] geometryB)
        {
            if (geometryA.Length == 1 && geometryB.Length == 1)
                return new List<CollisionPair>() { new CollisionPair(0, 0) };

            foreach (var shape in geometryA)
                shape.SetAABB(AABB.GetGeometryAABB(shape, shape));

            foreach (var shape in geometryB)
                shape.SetAABB(AABB.GetGeometryAABB(shape, shape));

            var geometryBoxesA = Array.ConvertAll(geometryA, x => x.AABBox);
            var geometryBoxesB = Array.ConvertAll(geometryB, x => x.AABBox);
            
            return broadPhaseCollisionEngine.Execute(geometryBoxesA, geometryBoxesB, 0.000001);
        }

        private List<CollisionPointStructure> SoftBodyCollisionStep(
            ISoftShape softShapeA,
            ISoftShape softShapeB)
        {
            var result = new List<CollisionPointStructure>();

            var shapeA = (IShape)softShapeA;
            var shapeB = (IShape)softShapeB;

            var convexDecompositionA = new ConvexDecompositionEngine(
                shapeA.AABBox,
                Array.ConvertAll(softShapeA.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex, item.ID)),
                softShapeA.DecompositionParameter);

            var convexDecompositionB = new ConvexDecompositionEngine(
                shapeB.AABBox,
                Array.ConvertAll(softShapeB.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex, item.ID)),
                softShapeB.DecompositionParameter);

            List<ShapeDecompositionOutput> decompConvexShapeA = convexDecompositionA.Execute().GetConvexShapeList(true);

            List<ShapeDecompositionOutput> decompConvexShapeB = convexDecompositionB.Execute().GetConvexShapeList(true);
            
            AABB[][] boxCollision = new AABB[2][];

            boxCollision[0] = Array.ConvertAll(decompConvexShapeA.ToArray(), x => x.Region);
            boxCollision[1] = Array.ConvertAll(decompConvexShapeB.ToArray(), x => x.Region);

            List<CollisionPair> collisionPair = broadPhaseCollisionEngine.Execute(
                boxCollision[0],
                boxCollision[1],
                parameters.CollisionDistance);

            var lockMe = new object();

            Parallel.ForEach(
                collisionPair,
                new ParallelOptions { MaxDegreeOfParallelism = parameters.MaxThreadNumber },
                pair =>
                {
                    CollisionPointStructure collisionPointStruct = SoftBodyNarrowPhase(
                        decompConvexShapeA[pair.objectIndexA],
                        decompConvexShapeB[pair.objectIndexB],
                        shapeA.ID,
                        shapeB.ID);

                    if (collisionPointStruct != null)
                    {
                        lock (lockMe)
                        {
                            result.Add(collisionPointStruct);
                        }
                    }
                });

            return result;
        }
                
        private CollisionPointStructure SoftBodyNarrowPhase(
            ShapeDecompositionOutput A,
            ShapeDecompositionOutput B,
            int ID_A,
            int ID_B)
        {
            List<CollisionPointStructure> collisionPointStructure = ConvexCollisionStep(A, B, ID_A, ID_B, parameters.CollisionDistance);

            if (collisionPointStructure.Count > 1)
            {
                List<CollisionPointBaseStructure> baseStructure = new List<CollisionPointBaseStructure>();

                foreach (CollisionPointStructure cps in collisionPointStructure)
                {
                    if (cps != null)
                        baseStructure.AddRange(cps.CollisionPointBase);
                }

                collisionPointStructure[0].SetBaseCollisionPoint(baseStructure.ToArray());
            }

            if (collisionPointStructure.Count > 0)
                return collisionPointStructure[0];

            return null;
        }

        private List<CollisionPointStructure> ConvexCollisionStep(
            ShapeDecompositionOutput A,
            ShapeDecompositionOutput B,
            int ID_A,
            int ID_B,
            double collisionDistance)
        {
            VertexProperties[] vertexObjA = Array.ConvertAll(A.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));
            VertexProperties[] vertexObjB = Array.ConvertAll(B.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));
                        
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            var cps = convexBodyNarrowPhase.Execute(vertexObjA, vertexObjB, ID_A, ID_B, collisionDistance);
                        
            if (cps != null)
                collisionPointStructure.Add(cps);

            return collisionPointStructure;
        }

        //Manca gestione compuondShape
        private List<CollisionPointStructure> Rigid_SoftBodyCollisionDetection(
            IShape rigidShape,
            ISoftShape softShape)
        {
            var shapeSoft = (IShape)softShape;

            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            ConvexDecompositionEngine convexDecomposition = new ConvexDecompositionEngine(
                shapeSoft.AABBox,
                Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex, item.ID)),
                softShape.DecompositionParameter);

            List<ShapeDecompositionOutput> shapeOutput = convexDecomposition.Execute().GetIntersectedShape(
                rigidShape.AABBox,
                parameters.CollisionDistance,
                true);
                        
            if (shapeOutput != null)
            {
                IGeometry[] convexShapeGeometry = ShapeDefinition.Helper.GetGeometry(rigidShape);

                foreach (var convexGeometry in convexShapeGeometry)
                {
                    VertexProperties[] convexVertexObj = Helper.SetVertexPosition(convexGeometry);

                    foreach (var softConvexShape in shapeOutput)
                    {
                        VertexProperties[] vertexObjSoftShape = Array.ConvertAll(softConvexShape.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));

                        var cps = convexBodyNarrowPhase.Execute(convexVertexObj, vertexObjSoftShape, rigidShape.ID, shapeSoft.ID, parameters.CollisionDistance);

                        if (cps != null)
                            collisionPointStructure.Add(cps);
                    }
                }
            }

            return collisionPointStructure;
        }

        #endregion

    }
}
