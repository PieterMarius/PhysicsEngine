﻿using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.CollisionEngine.SoftBody;
using System.Linq;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace SharpPhysicsEngine.CollisionEngine
{
	public class CollisionDetectionEngine: ICollisionEngine
	{
		#region Private Fields

		private const double normalTolerance = 1E-15;
		
		private GJK collisionEngine;
		private EPA compenetrationCollisionEngine;
		private readonly CollisionEngineParameters collisionEngineParameters;
		private readonly SweepAndPruneEngine sweepAndPruneEngine;
		//TODO eliminare
		private readonly SoftBodyCollisionDetection softBodyCollisionDetection;

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

			sweepAndPruneEngine = new SweepAndPruneEngine (collisionEngineParameters);
			softBodyCollisionDetection = new SoftBodyCollisionDetection();

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
		public List<CollisionPointStructure> Execute(IShape[] shapes)
		{
			return (collisionEngineParameters.ActivateSweepAndPrune) ?
					SweepAndPruneBroadPhase(shapes) :
					BruteForceBroadPhase(shapes);
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
				var mpg = new ManifoldPointsGenerator(
												collisionEngineParameters.ManifoldPointNumber,
												collisionEngineParameters.GJKManifoldTolerance,
												collisionEngineParameters.ManifoldProjectionTolerance);

				if (gjkOutput.CollisionNormal.Length() < normalTolerance)
					return null;

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
					Array.ConvertAll(vertexObjA, x => x.Vertex),
					Array.ConvertAll(vertexObjB, x => x.Vertex),
					gjkOutput.CollisionPoint);

				return new CollisionPointStructure(
					ID_A,
					ID_B,
					new CollisionPointBaseStructure(
						gjkOutput.CollisionDistance,
						gjkOutput.Intersection,
						gjkOutput.CollisionPoint,
						collisionPointsList.ToArray()));
			}

			if (gjkOutput.Intersection)
			{
				EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
												vertexObjA,
												vertexObjB,
												gjkOutput.SupportTriangles,
												gjkOutput.Centroid);

				if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
					return null;

				var mpg = new ManifoldPointsGenerator(
												  collisionEngineParameters.ManifoldPointNumber,
												  collisionEngineParameters.EPAManifoldTolerance,
												  collisionEngineParameters.ManifoldProjectionTolerance);

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
															   Array.ConvertAll(vertexObjA, x => x.Vertex),
															   Array.ConvertAll(vertexObjB, x => x.Vertex),
															   epaOutput.CollisionPoint);

				return new CollisionPointStructure(
					ID_A,
					ID_B,
					new CollisionPointBaseStructure(
						epaOutput.CompenetrationDistance,
						gjkOutput.Intersection,
						epaOutput.CollisionPoint,
						collisionPointsList.ToArray()));
			}

			return null;
		}

		private CollisionPointStructure NarrowPhase(
			IShape A,
			IShape B)
		{
			List<CollisionPointStructure> collisionPointStructure = GetCollisionPointStructure(
				A, 
				B);
			
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
				
		private List<CollisionPointStructure> BruteForceBroadPhase(IShape[] shapes)
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

		private List<CollisionPointStructure> SweepAndPruneBroadPhase(IShape[] shapes)
		{
			var result = new List<CollisionPointStructure> ();

			AABB[][] boxs = GetAABB(shapes);
			
			List<CollisionPair> collisionPair = sweepAndPruneEngine.Execute (boxs, CollisionDistance);

			var lockMe = new object();

			Parallel.ForEach(
				collisionPair,
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
				pair =>
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
				});

			return result;
		}

		private AABB[][] GetAABB(IShape[] shapes)
		{
			AABB[][] boxs = new AABB[shapes.Length][];

			foreach (var item in shapes.Select((shape, i) => new { shape, i }))
			{
				IConvexShape shape = item.shape as IConvexShape;
				if (shape != null)
				{
					boxs[item.i] = new AABB[1];
					boxs[item.i][0] = shape.ObjectGeometry.AABBox;
					continue;
				}

				ICompoundShape compoundShape = item.shape as ICompoundShape;
				if (compoundShape != null)
				{
					AABB[] bufBox = Array.ConvertAll(compoundShape.ObjectGeometry, x => x.AABBox);
					boxs[item.i] = bufBox;
					continue;
				}

				ISoftShape softShape = item.shape as ISoftShape;
				if (softShape != null)
				{
					boxs[item.i] = new AABB[1];
					boxs[item.i][0] = softShape.AABBox;
					continue;
				}
			}

			return boxs;
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
				//Manca gestione compuondShape
				collisionPointStructure.AddRange(RigidVSSoftBodyCollisionDetection((IConvexShape)A, softShapeB));
			}
			else if (softShapeA != null && 
					 softShapeB == null)
			{
				///Manca gestione compuondShape
				collisionPointStructure.AddRange(RigidVSSoftBodyCollisionDetection((IConvexShape)B, softShapeA));
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
		private List<CollisionPointStructure> RigidVSSoftBodyCollisionDetection(
			IConvexShape convexShape,
			ISoftShape softShape)
		{
			List<CollisionPointStructure> result = new List<CollisionPointStructure>();
						
			List<ShapeDecompositionOutput> shapeOutput = softShape.ConvexDecomposition.GetIntersectedShape(
				convexShape.ObjectGeometry.AABBox,
				softShape.AABBox,
				Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.GetID())),
				0.2);

			VertexProperties[] vertexObjConvexShape = Helper.SetVertexPosition(convexShape.ObjectGeometry);

			for (int i = 0; i < shapeOutput.Count; i++)
			{
				VertexProperties[] vertexObjSoftShape = Array.ConvertAll(shapeOutput[i].Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));

				GJKOutput gjkOutput = collisionEngine.Execute(vertexObjConvexShape, vertexObjSoftShape);

				if (!gjkOutput.Intersection &&
					gjkOutput.CollisionDistance <= CollisionDistance)
				{
					var mpg = new ManifoldPointsGenerator(
													collisionEngineParameters.ManifoldPointNumber,
													collisionEngineParameters.GJKManifoldTolerance,
													collisionEngineParameters.ManifoldProjectionTolerance);

					if (gjkOutput.CollisionNormal.Length() < normalTolerance)
						return null;

					List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
						Array.ConvertAll(vertexObjConvexShape, x => x.Vertex),
						Array.ConvertAll(vertexObjSoftShape, x => x.Vertex),
						gjkOutput.CollisionPoint);

					result.Add(new CollisionPointStructure(
						((IShape)convexShape).GetID(),
						((IShape)softShape).GetID(),
						new CollisionPointBaseStructure(
							gjkOutput.CollisionDistance,
							gjkOutput.Intersection,
							gjkOutput.CollisionPoint,
							collisionPointsList.ToArray())));
				}

				if (gjkOutput.Intersection)
				{
					EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
													vertexObjConvexShape,
													vertexObjSoftShape,
													gjkOutput.SupportTriangles,
													gjkOutput.Centroid);

					if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
						return null;

					var mpg = new ManifoldPointsGenerator(
													  collisionEngineParameters.ManifoldPointNumber,
													  collisionEngineParameters.EPAManifoldTolerance,
													  collisionEngineParameters.ManifoldProjectionTolerance);

					List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
																   Array.ConvertAll(vertexObjConvexShape, x => x.Vertex),
																   Array.ConvertAll(vertexObjSoftShape, x => x.Vertex),
																   epaOutput.CollisionPoint);

					result.Add(new CollisionPointStructure(
						((IShape)convexShape).GetID(),
						((IShape)softShape).GetID(),
						new CollisionPointBaseStructure(
							epaOutput.CompenetrationDistance,
							gjkOutput.Intersection,
							epaOutput.CollisionPoint,
							collisionPointsList.ToArray())));
				}
			}

			return result;
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

			for (int geometryIndexA = 0; geometryIndexA < geometryA.Length; geometryIndexA++)
			{
				for (int geometryIndexB = 0; geometryIndexB < geometryB.Length; geometryIndexB++)
				{
					VertexProperties[] vertexObjA = Helper.SetVertexPosition(geometryA[geometryIndexA]);
					VertexProperties[] vertexObjB = Helper.SetVertexPosition(geometryB[geometryIndexB]);

					GJKOutput gjkOutput = collisionEngine.Execute(vertexObjA, vertexObjB);

					CollisionPointStructure collision = NarrowPhaseCollisionDetection(
						gjkOutput,
						vertexObjA,
						vertexObjB,
						A.GetID(),
						B.GetID());

					if (collision != null)
						collisionPointStructure.Add(collision);
				}
			}

			return collisionPointStructure;
		}

		#region Soft Body Collision Detection

		private List<CollisionPointStructure> SoftBodyCollisionStep(
			ISoftShape softShapeA,
			ISoftShape softShapeB)
		{
			var result = new List<CollisionPointStructure>();

			List<ShapeDecompositionOutput> convexShapeA = softShapeA.ConvexDecomposition.GetConvexShapeList(
				Array.ConvertAll(softShapeA.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.GetID())),
				0.2);

			List<ShapeDecompositionOutput> convexShapeB = softShapeB.ConvexDecomposition.GetConvexShapeList(
				Array.ConvertAll(softShapeB.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.GetID())),
				0.2);

			AABB[][] boxCollision = new AABB[2][];

			boxCollision[0] = Array.ConvertAll(convexShapeA.ToArray(), x => x.Region);
			boxCollision[1] = Array.ConvertAll(convexShapeB.ToArray(), x => x.Region);

			List<CollisionPair> collisionPair = new List<CollisionPair>();

			for (int i = 0; i < boxCollision[0].Length; i++)
			{
				for (int j = 0; j < boxCollision[1].Length; j++)
				{
					if (AABB.Intersect(boxCollision[0][i], boxCollision[1][j], CollisionDistance))
						collisionPair.Add(new CollisionPair(i, j));
				}
			}

			var lockMe = new object();

			Parallel.ForEach(
				collisionPair,
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
				pair =>
				{
					CollisionPointStructure collisionPointStruct = SoftBodyNarrowPhase(
						convexShapeA[pair.objectIndexA],
						convexShapeB[pair.objectIndexB],
						((IShape)softShapeA).GetID(),
						((IShape)softShapeA).GetID());

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
			List<CollisionPointStructure> collisionPointStructure = RigidBodyCollisionStep(A, B, ID_A, ID_B);
			
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

		private List<CollisionPointStructure> RigidBodyCollisionStep(
			ShapeDecompositionOutput A,
			ShapeDecompositionOutput B,
			int ID_A,
			int ID_B)
		{
			List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

			VertexProperties[] vertexObjA = Array.ConvertAll(A.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));
			VertexProperties[] vertexObjB = Array.ConvertAll(B.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));

			GJKOutput gjkOutput = collisionEngine.Execute(vertexObjA, vertexObjB);

			#region Narrow Phase

			if (!gjkOutput.Intersection &&
				gjkOutput.CollisionDistance <= CollisionDistance)
			{
				var mpg = new ManifoldPointsGenerator(
												collisionEngineParameters.ManifoldPointNumber,
												collisionEngineParameters.GJKManifoldTolerance,
												collisionEngineParameters.ManifoldProjectionTolerance);

				if (gjkOutput.CollisionNormal.Length() < normalTolerance)
					return null;

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
					Array.ConvertAll(vertexObjA, x => x.Vertex),
					Array.ConvertAll(vertexObjB, x => x.Vertex),
					gjkOutput.CollisionPoint);

				collisionPointStructure.Add(new CollisionPointStructure(
					ID_A,
					ID_B,
					new CollisionPointBaseStructure(
						gjkOutput.CollisionDistance,
						gjkOutput.Intersection,
						gjkOutput.CollisionPoint,
						collisionPointsList.ToArray())));
			}

			if (gjkOutput.Intersection)
			{
				EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
												vertexObjA,
												vertexObjB,
												gjkOutput.SupportTriangles,
												gjkOutput.Centroid);

				if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
					return null;

				var mpg = new ManifoldPointsGenerator(
												  collisionEngineParameters.ManifoldPointNumber,
												  collisionEngineParameters.EPAManifoldTolerance,
												  collisionEngineParameters.ManifoldProjectionTolerance);

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
															   Array.ConvertAll(vertexObjA, x => x.Vertex),
															   Array.ConvertAll(vertexObjB, x => x.Vertex),
															   epaOutput.CollisionPoint);

				collisionPointStructure.Add(new CollisionPointStructure(
					ID_A,
					ID_B,
					new CollisionPointBaseStructure(
						epaOutput.CompenetrationDistance,
						gjkOutput.Intersection,
						epaOutput.CollisionPoint,
						collisionPointsList.ToArray())));
			}

			#endregion
			
			return collisionPointStructure;

		}

		#endregion

		#endregion

	}
}

