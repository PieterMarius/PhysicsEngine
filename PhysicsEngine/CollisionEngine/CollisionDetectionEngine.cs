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

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using SharpPhysicsEngine.ShapeDefinition;
using System.Linq;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal class CollisionDetectionEngine: ICollisionEngine
	{
		#region Private Fields

		private const double normalTolerance = 1E-15;
		
		private GJK collisionEngine;
		private EPA compenetrationCollisionEngine;
		private readonly CollisionEngineParameters collisionEngineParameters;
		private readonly SweepAndPruneEngine sweepAndPruneEngine;
		private readonly ManifoldPointsGenerator manifoldGJKPointsGenerator;
        private readonly ManifoldPointsGenerator manifoldEPAPointsGenerator;
        
		private double CollisionDistance;

		#endregion

		#region Constructor

		public CollisionDetectionEngine (
			CollisionEngineParameters collisionEngineParameters,
			double collisionDistance)
		{
			this.collisionEngineParameters = collisionEngineParameters;

			collisionEngine = new GJK (
				collisionEngineParameters.MaxGJKIteration,
				collisionEngineParameters.Precision,
				collisionEngineParameters.GJKManifoldTolerance,
				collisionEngineParameters.ManifoldPointNumber);

			compenetrationCollisionEngine = new EPA (
				collisionEngineParameters.MaxEPAIteration,
				collisionEngineParameters.Precision,
				collisionEngineParameters.EPAManifoldTolerance,
				collisionEngineParameters.ManifoldPointNumber);

            manifoldGJKPointsGenerator = new ManifoldPointsGenerator(
                collisionEngineParameters.ManifoldPointNumber,
                collisionEngineParameters.GJKManifoldTolerance,
                collisionEngineParameters.ManifoldProjectionTolerance);

            manifoldEPAPointsGenerator = new ManifoldPointsGenerator(
                collisionEngineParameters.ManifoldPointNumber,
				collisionEngineParameters.EPAManifoldTolerance,
				collisionEngineParameters.ManifoldProjectionTolerance);

            sweepAndPruneEngine = new SweepAndPruneEngine (collisionEngineParameters);
			
			CollisionDistance = collisionDistance;
		}

		#endregion

		#region Public Methods

		#region Interface ICollisionEngine

		/// <summary>
		/// Runs the test collision.
		/// </summary>
		/// <returns>The test collision.</returns>
		/// <param name="shapes">Objects.</param>
		/// <param name="minDistance">Minimum distance.</param>
		public List<CollisionPointStructure> Execute(
            IShape[] shapes,
            HashSet<HashSetStruct> ignoreList)
		{
            if (ignoreList == null)
                ignoreList = new HashSet<HashSetStruct>();

			return (collisionEngineParameters.ActivateSweepAndPrune) ?
					SweepAndPruneBroadPhase(shapes, ignoreList) :
					BruteForceBroadPhase(shapes, ignoreList);
		}

		public void SetCollisionDistance(double collisionDistance)
		{
			CollisionDistance = collisionDistance;
		}

		/// <summary>
		/// Gets the engine parameters.
		/// </summary>
		/// <returns>The engine parameters.</returns>
		public CollisionEngineParameters GetEngineParameters()
		{
			return collisionEngineParameters;
		}

		#endregion

		#endregion

		#region Private Methods

		private CollisionPointStructure NarrowPhase(
			IShape A,
			IShape B)
		{
			List<CollisionPointStructure> collisionPointStructure = GetCollisionPointStructure(A, B);
			
			if (collisionPointStructure.Count > 1)
			{
				List<CollisionPointBaseStructure> baseStructure = new List<CollisionPointBaseStructure>();

				foreach (CollisionPointStructure cps in collisionPointStructure)
				{
					if(cps != null)
						baseStructure.AddRange(cps.CollisionPointBase);
				}

				collisionPointStructure[0].SetBaseCollisionPoint(baseStructure.ToArray());
			}

			if (collisionPointStructure.Count > 0)
				return collisionPointStructure[0];

			return null;
		}
				
		private List<CollisionPointStructure> BruteForceBroadPhase(
            IShape[] shapes,
            HashSet<HashSetStruct> ignoreList)
		{
			var result = new List<CollisionPointStructure> ();

			var lockMe = new object();

			if (shapes != null)
			{
				Parallel.For(0,
					shapes.Length,
					new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
					i =>
					{
						if (shapes[i] != null)
						{
							for (int j = i + 1; j < shapes.Length; j++)
							{
								if (shapes[j] != null)
								{
                                    if (ignoreList.Contains(new HashSetStruct(shapes[i].ID, shapes[j].ID)))
                                        continue;

									CollisionPointStructure collisionPointStruct = NarrowPhase(
																				  shapes[i],
																				  shapes[j]);

									lock (lockMe)
									{
										if (collisionPointStruct != null)
											result.Add(collisionPointStruct);
									}
								}
							}
						}
					});
			}
			
			return result;
		}

		private List<CollisionPointStructure> SweepAndPruneBroadPhase(
            IShape[] shapes,
            HashSet<HashSetStruct> ignoreList)
		{
			var result = new List<CollisionPointStructure> ();

			AABB[] boxs = GetAABB(shapes);
			
			List<CollisionPair> collisionPair = sweepAndPruneEngine.Execute (boxs, CollisionDistance);

			var lockMe = new object();

			Parallel.ForEach(
				collisionPair,
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
				pair =>
				{
                    if (!ignoreList.Contains(new HashSetStruct(shapes[pair.objectIndexA].ID, shapes[pair.objectIndexB].ID)))
                    {
                        CollisionPointStructure collisionPointStruct = NarrowPhase(
                            shapes[pair.objectIndexA],
                            shapes[pair.objectIndexB]);

                        if (collisionPointStruct != null)
                        {
                            lock (lockMe)
                            {
                                result.Add(collisionPointStruct);
                            }
                        }
                    }
				});

			return result;
		}

		private AABB[] GetAABB(IShape[] shapes)
		{
            return Array.ConvertAll(shapes, x => x.AABBox);
		}

		private List<CollisionPointStructure> GetCollisionPointStructure(
			IShape A,
			IShape B)
		{
			List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

			ISoftShape softShapeA = A as ISoftShape;
			ISoftShape softShapeB = B as ISoftShape;

			if (softShapeA == null &&
				softShapeB == null)
			{
				return RigidBodyCollisionStep(A, B);
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

		//Manca gestione compuondShape
		private List<CollisionPointStructure> Rigid_SoftBodyCollisionDetection(
			IShape rigidShape,
			ISoftShape softShape)
		{
            var shapeSoft = (IShape)softShape;
            
			List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            ShapeConvexDecomposition convexDecomposition = new ShapeConvexDecomposition(shapeSoft.AABBox, softShape.Triangle);

            List<ShapeDecompositionOutput> shapeOutput = convexDecomposition.GetIntersectedShape(
				rigidShape.AABBox,
                shapeSoft.AABBox,
				Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.ID)),
				softShape.DecompositionParameter,
                CollisionDistance);

            if (shapeOutput != null)
            {
                IGeometry[] convexShapeGeometry = ShapeDefinition.Helper.GetGeometry(rigidShape);

                foreach (var convexGeometry in convexShapeGeometry)
                {
                    VertexProperties[] convexVertexObj = Helper.SetVertexPosition(convexGeometry);

                    foreach (var softConvexShape in shapeOutput)
                    {
                        VertexProperties[] vertexObjSoftShape = Array.ConvertAll(softConvexShape.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));

                        GJKOutput gjkOutput = collisionEngine.Execute(convexVertexObj, vertexObjSoftShape);

                        var cps = NarrowPhaseCollisionDetection(
                            gjkOutput, 
                            convexVertexObj, 
                            vertexObjSoftShape,
                            rigidShape.ID,
                            shapeSoft.ID);

                        if (cps != null)
                            collisionPointStructure.Add(cps);
                    }
                }
            }
            
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
			IShape B)
		{
			List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

			IGeometry[] geometryA = ShapeDefinition.Helper.GetGeometry(A);
			IGeometry[] geometryB = ShapeDefinition.Helper.GetGeometry(B);

            List<CollisionPair> collisionPair = CheckGeometryAABB(
                geometryA,
                geometryB);
                        
            foreach(var collidingPair in collisionPair)
            {
                VertexProperties[] vertexObjA = Helper.SetVertexPosition(geometryA[collidingPair.objectIndexA]);
                VertexProperties[] vertexObjB = Helper.SetVertexPosition(geometryB[collidingPair.objectIndexB]);

                GJKOutput gjkOutput = collisionEngine.Execute(vertexObjA, vertexObjB);

                CollisionPointStructure collision = NarrowPhaseCollisionDetection(
                    gjkOutput,
                    vertexObjA,
                    vertexObjB,
                    A.ID,
                    B.ID);

                if (collision != null)
                    collisionPointStructure.Add(collision);
            }
                       
			return collisionPointStructure;
		}

        private List<CollisionPair> CheckGeometryAABB(
            IGeometry[] geometryA,
            IGeometry[] geometryB)
        {
            var geometryBoxesA = Array.ConvertAll(geometryA, x => x.AABBox);
            var geometryBoxesB = Array.ConvertAll(geometryB, x => x.AABBox);

            if(geometryBoxesA.Length == 1 &&
               geometryBoxesB.Length == 1)
                return new List<CollisionPair>() { new CollisionPair(0, 0) };
            
            return sweepAndPruneEngine.Execute(geometryBoxesA, geometryBoxesB, CollisionDistance);
        }

		#region Soft Body Collision Detection

		private List<CollisionPointStructure> SoftBodyCollisionStep(
			ISoftShape softShapeA,
			ISoftShape softShapeB)
		{
			var result = new List<CollisionPointStructure>();

            var shapeA = (IShape)softShapeA;
            var shapeB = (IShape)softShapeB;

            ShapeConvexDecomposition convexDecompositionA = new ShapeConvexDecomposition(shapeA.AABBox, softShapeA.Triangle);
            ShapeConvexDecomposition convexDecompositionB = new ShapeConvexDecomposition(shapeB.AABBox, softShapeB.Triangle);

            List<ShapeDecompositionOutput> decompConvexShapeA = convexDecompositionA.GetConvexShapeList(
				Array.ConvertAll(softShapeA.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.ID)),
				softShapeA.DecompositionParameter);

			List<ShapeDecompositionOutput> decompConvexShapeB = convexDecompositionB.GetConvexShapeList(
				Array.ConvertAll(softShapeB.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.ID)),
				softShapeB.DecompositionParameter);

			AABB[][] boxCollision = new AABB[2][];

			boxCollision[0] = Array.ConvertAll(decompConvexShapeA.ToArray(), x => x.Region);
			boxCollision[1] = Array.ConvertAll(decompConvexShapeB.ToArray(), x => x.Region);

            List<CollisionPair> collisionPair = sweepAndPruneEngine.Execute(
                boxCollision[0], 
                boxCollision[1], 
                CollisionDistance);

			var lockMe = new object();

            Parallel.ForEach(
				collisionPair,
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
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
			List<CollisionPointStructure> collisionPointStructure = ConvexCollisionStep(A, B, ID_A, ID_B);
			
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
			int ID_B)
		{
			VertexProperties[] vertexObjA = Array.ConvertAll(A.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));
			VertexProperties[] vertexObjB = Array.ConvertAll(B.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));

			GJKOutput gjkOutput = collisionEngine.Execute(vertexObjA, vertexObjB);
                        
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            var cps = NarrowPhaseCollisionDetection(gjkOutput, vertexObjA, vertexObjB, ID_A, ID_B);

            if (cps != null)
                collisionPointStructure.Add(cps);
                        			
			return collisionPointStructure;
		}

		private CollisionPointStructure NarrowPhaseCollisionDetection(
			GJKOutput gjkOutput,
			VertexProperties[] vertexObjA,
			VertexProperties[] vertexObjB,
			int ID_A,
			int ID_B)
		{
            if (!gjkOutput.Intersection &&
				gjkOutput.CollisionDistance <= CollisionDistance)
			{
				if (gjkOutput.CollisionNormal.Length() < normalTolerance)
					return null;

                List<CollisionPoint> collisionPointsList = manifoldGJKPointsGenerator.GetManifoldPoints(
					Array.ConvertAll(vertexObjA, x => x.Vertex),
					Array.ConvertAll(vertexObjB, x => x.Vertex),
					gjkOutput.CollisionPoint);

                var collisionPointBaseStr = new CollisionPointBaseStructure(
                        gjkOutput.CollisionPoint,
                        collisionPointsList.ToArray());
                
                return new CollisionPointStructure(
					ID_A,
					ID_B,
                    collisionPointBaseStr);
			}
			else if (gjkOutput.Intersection)
			{
				EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
												vertexObjA,
												vertexObjB,
												gjkOutput.SupportTriangles,
												gjkOutput.Centroid);

				if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
					return null;

				List<CollisionPoint> collisionPointsList = manifoldEPAPointsGenerator.GetManifoldPoints(
															   Array.ConvertAll(vertexObjA, x => x.Vertex),
															   Array.ConvertAll(vertexObjB, x => x.Vertex),
															   epaOutput.CollisionPoint);

                var collisionPointBaseStr = new CollisionPointBaseStructure(
                        epaOutput.CollisionPoint,
                        collisionPointsList.ToArray());
                
                return new CollisionPointStructure(
					ID_A,
					ID_B,
                    collisionPointBaseStr);
			}

			return null;
		}
        
        #endregion

        #endregion

    }
}

