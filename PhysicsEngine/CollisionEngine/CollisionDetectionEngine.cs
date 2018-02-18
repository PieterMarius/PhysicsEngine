using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using SharpPhysicsEngine.ShapeDefinition;
using System.Linq;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

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
                if (item.shape is IConvexShape shape)
                {
                    boxs[item.i] = new AABB[1];
                    boxs[item.i][0] = shape.ObjectGeometry.AABBox;
                    continue;
                }

                if (item.shape is ICompoundShape compoundShape)
                {
                    AABB[] bufBox = Array.ConvertAll(compoundShape.ObjectGeometry, x => x.AABBox);
                    boxs[item.i] = bufBox;
                    continue;
                }

                if (item.shape is ISoftShape softShape)
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
				collisionPointStructure.AddRange(Rigid_SoftBodyCollisionDetection((IConvexShape)A, softShapeB));
			}
			else if (softShapeA != null && 
					 softShapeB == null)
			{
				///Manca gestione compuondShape
				collisionPointStructure.AddRange(Rigid_SoftBodyCollisionDetection((IConvexShape)B, softShapeA));
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
			IConvexShape convexShape,
			ISoftShape softShape)
		{
			List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();
		    
            	List<ShapeDecompositionOutput> shapeOutput = softShape.ConvexDecomposition.GetIntersectedShape(
				convexShape.ObjectGeometry.AABBox,
				softShape.AABBox,
				Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.ID)),
				softShape.DecompositionParameter,
                CollisionDistance);

            if (shapeOutput != null)
            {
                IGeometry[] convexShapeGeometry = ShapeDefinition.Helper.GetGeometry((IShape)convexShape);

                var ID_A = ((IShape)convexShape).ID;
                var ID_B = ((IShape)softShape).ID;

                foreach (var convexGeometry in convexShapeGeometry)
                {
                    VertexProperties[] convexVertexObj = Helper.SetVertexPosition(convexGeometry);

                    foreach (var softConvexShape in shapeOutput)
                    {
                        VertexProperties[] vertexObjSoftShape = Array.ConvertAll(softConvexShape.Vertex3Idx.ToArray(), x => new VertexProperties(x.Vector3, x.ID));

                        GJKOutput gjkOutput = collisionEngine.Execute(convexVertexObj, vertexObjSoftShape);

                        var cps = NarrowPhaseCollisionDetection(gjkOutput, convexVertexObj, vertexObjSoftShape, ID_A, ID_B);

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

            int ID_A = A.ID;
            int ID_B = B.ID;

            foreach (var shapeA in geometryA)
            {
                foreach (var shapeB in geometryB)
                {
                    VertexProperties[] vertexObjA = Helper.SetVertexPosition(shapeA);
                    VertexProperties[] vertexObjB = Helper.SetVertexPosition(shapeB);

                    GJKOutput gjkOutput = collisionEngine.Execute(vertexObjA, vertexObjB);

                    CollisionPointStructure collision = NarrowPhaseCollisionDetection(
                        gjkOutput,
                        vertexObjA,
                        vertexObjB,
                        ID_A,
                        ID_B);

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

			List<ShapeDecompositionOutput> decompConvexShapeA = softShapeA.ConvexDecomposition.GetConvexShapeList(
				Array.ConvertAll(softShapeA.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.ID)),
				softShapeA.DecompositionParameter);

			List<ShapeDecompositionOutput> decompConvexShapeB = softShapeB.ConvexDecomposition.GetConvexShapeList(
				Array.ConvertAll(softShapeB.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(), item.ID)),
				softShapeB.DecompositionParameter);

			AABB[][] boxCollision = new AABB[2][];

			boxCollision[0] = Array.ConvertAll(decompConvexShapeA.ToArray(), x => x.Region);
			boxCollision[1] = Array.ConvertAll(decompConvexShapeB.ToArray(), x => x.Region);

			List<CollisionPair> collisionPair = new List<CollisionPair>();

            //TODO Utilizzare la funzione qui descritta
            //List<CollisionPair> collisionPair = sweepAndPruneEngine.Execute(boxCollision, CollisionDistance);

            for (int i = 0; i < boxCollision[0].Length; i++)
			{
				for (int j = 0; j < boxCollision[1].Length; j++)
				{
					if (AABB.Intersect(boxCollision[0][i], boxCollision[1][j], CollisionDistance))
						collisionPair.Add(new CollisionPair(i, j));
				}
			}

			var lockMe = new object();

            int ID_A = ((IShape)softShapeA).ID;
            int ID_B = ((IShape)softShapeB).ID;


            Parallel.ForEach(
				collisionPair,
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
				pair =>
				{
					CollisionPointStructure collisionPointStruct = SoftBodyNarrowPhase(
						decompConvexShapeA[pair.objectIndexA],
						decompConvexShapeB[pair.objectIndexB],
						ID_A,
						ID_B);

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

            #region Narrow Phase

            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            var cps = NarrowPhaseCollisionDetection(gjkOutput, vertexObjA, vertexObjB, ID_A, ID_B);

            if (cps != null)
                collisionPointStructure.Add(cps);
            
            #endregion
			
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

				List<CollisionPoint> collisionPointsList = manifoldEPAPointsGenerator.GetManifoldPoints(
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

		#endregion

		#endregion

	}
}

