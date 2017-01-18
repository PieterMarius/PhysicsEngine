using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using ShapeDefinition;
using PhysicsEngineMathUtility;
using CollisionEngine.SoftBody;

namespace CollisionEngine
{
	public class CollisionDetectionEngine: ICollisionEngine
	{
        #region Private Fields

        const double normalTolerance = 1E-15;
        
        GJK collisionEngine;
		EPA compenetrationCollisionEngine;
		readonly CollisionEngineParameters collisionEngineParameters;
		readonly SweepAndPruneEngine sweepAndPruneEngine;
        readonly SoftBodyCollisionDetection softBodyCollisionDetection;

		#endregion

		#region Constructor

		public CollisionDetectionEngine (
			CollisionEngineParameters collisionEngineParameters)
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
		}

		#endregion

		#region Public Methods

		#region Interface ICollisionEngine

		/// <summary>
		/// Runs the test collision.
		/// </summary>
		/// <returns>The test collision.</returns>
		/// <param name="objects">Objects.</param>
		/// <param name="minDistance">Minimum distance.</param>
		public List<CollisionPointStructure> Execute(
			IShape[] objects,
			double minDistance)
		{
			if (collisionEngineParameters.ActivateSweepAndPrune) 
			{
				return SweepAndPruneBroadPhase(
					objects,
					minDistance);
			} 
			else 
			{
				return BruteForceBroadPhase(
					objects,
					minDistance);
			}
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

        private CollisionPointStructure NarrowPhaseCollisionControl(
            GJKOutput gjkOutput,
            IGeometry A,
            IGeometry B,
            int indexA,
            int indexB,
            double minDistance)
        {
            
            if (!gjkOutput.Intersection &&
                gjkOutput.CollisionDistance <= minDistance)
            {
                var mpg = new ManifoldPointsGenerator(
                                                collisionEngineParameters.ManifoldPointNumber,
                                                collisionEngineParameters.GJKManifoldTolerance,
                                                collisionEngineParameters.ManifoldProjectionTolerance);

                if (gjkOutput.CollisionNormal.Length() < normalTolerance)
                    return null;

                List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
                                                                A,
                                                                B,
                                                                gjkOutput.CollisionPoint);

                return new CollisionPointStructure(
                    indexA,
                    indexB,
                    new CollisionPointBaseStructure(
                        gjkOutput.CollisionDistance, 
                        gjkOutput.Intersection, 
                        gjkOutput.CollisionPoint,
                        collisionPointsList.ToArray()));
            }

            if (gjkOutput.Intersection)
            {
                EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
                                                A,
                                                B,
                                                gjkOutput.SupportTriangles,
                                                gjkOutput.Centroid);

                if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
                    return null;

                var mpg = new ManifoldPointsGenerator(
                                                  collisionEngineParameters.ManifoldPointNumber,
                                                  collisionEngineParameters.EPAManifoldTolerance,
                                                  collisionEngineParameters.ManifoldProjectionTolerance);

                List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
                                                               A,
                                                               B,
                                                               epaOutput.CollisionPoint);

                return new CollisionPointStructure(
                    indexA,
                    indexB,
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
			IShape B,
			int indexA,
			int indexB,
			double minDistance)
		{
            List<CollisionPointStructure> collisionPointStructure = GetCollisionPointStructure(A, B, indexA, indexB, minDistance);
            
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
			IShape[] objects,
			double minDistance)
		{
			var result = new List<CollisionPointStructure> ();

			var lockMe = new object();

            Parallel.For(0,
                objects.Length,
                new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
                i =>
                {
                    if (objects[i] != null)
                    {
                        for (int j = i + 1; j < objects.Length; j++)
                        {
                            if (objects[j] != null)
                            {
                                CollisionPointStructure collisionPointStruct = NarrowPhase(
                                                                              objects[i],
                                                                              objects[j],
                                                                              i,
                                                                              j,
                                                                              minDistance);

                                lock (lockMe)
                                {
                                    if (collisionPointStruct != null)
                                        result.Add(collisionPointStruct);
                                }
                            }
                        }
                    }
                });
			
			return result;
		}

		private List<CollisionPointStructure> SweepAndPruneBroadPhase(
			IShape[] objects,
			double minDistance)
		{
			var result = new List<CollisionPointStructure> ();

            AABB[][] boxs = GetAABBArray(objects);
            
            List<CollisionPair> collisionPair = sweepAndPruneEngine.Execute (boxs, minDistance);

            	var lockMe = new object();

            Parallel.ForEach(
                collisionPair,
                new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
                pair =>
                {
                    CollisionPointStructure collisionPointStruct = NarrowPhase(
                        objects[pair.objectIndexA],
                        objects[pair.objectIndexB],
                        pair.objectIndexA,
                        pair.objectIndexB,
                        minDistance);

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

        private AABB[][] GetAABBArray(IShape[] objects)
        {
            AABB[][] boxs = new AABB[objects.Length][];

            int index = 0;
            foreach (IShape shape in objects)
            {
                if (shape is IConvexShape)
                {
                    boxs[index] = new AABB[1];
                    boxs[index][0] = ((IConvexShape)shape).ObjectGeometry.AABBox;
                }
                else if (shape is ICompoundShape)
                {

                    AABB[] bufBox = Array.ConvertAll(((ICompoundShape)shape).ObjectGeometry, x => x.AABBox);
                    boxs[index] = bufBox;
                }
                else if (shape is ISoftShape)
                {
                    boxs[index] = new AABB[1];
                    boxs[index][0] = ((ISoftShape)shape).AABBox;
                }
                index++;
            }

            return boxs;
        }

        private List<CollisionPointStructure> GetCollisionPointStructure(
            IShape A,
            IShape B,
            int indexA,
            int indexB,
            double minDistance)
        {
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            if (!(A is ISoftShape) &&
                !(B is ISoftShape))
            {
                IGeometry[] geometryA = ShapeDefinition.Helper.GetGeometry(A);
                IGeometry[] geometryB = ShapeDefinition.Helper.GetGeometry(B);

                for (int geometryIndexA = 0; geometryIndexA < geometryA.Length; geometryIndexA++)
                {
                    for (int geometryIndexB = 0; geometryIndexB < geometryB.Length; geometryIndexB++)
                    {
                        GJKOutput gjkOutput = collisionEngine.Execute(
                            geometryA[geometryIndexA],
                            geometryB[geometryIndexB]);

                        CollisionPointStructure collision = NarrowPhaseCollisionControl(
                            gjkOutput,
                            geometryA[geometryIndexA],
                            geometryB[geometryIndexB],
                            indexA,
                            indexB,
                            minDistance);

                        if (collision != null)
                            collisionPointStructure.Add(collision);
                    }
                }
            }
            else if (A is ISoftShape && 
                     B is ISoftShape)
            {
                List<CollisionPointBaseStructure> baseCollisionList = new List<CollisionPointBaseStructure>();

                ISoftShape softShapeA = (ISoftShape)A;
                ISoftShape softShapeB = (ISoftShape)B;

                baseCollisionList.AddRange(softBodyCollisionDetection.SelfSoftBodyCollisionDetection(softShapeA, minDistance));
                baseCollisionList.AddRange(softBodyCollisionDetection.SelfSoftBodyCollisionDetection(softShapeB, minDistance));

                baseCollisionList.AddRange(softBodyCollisionDetection.SoftVsSoftBodyCollisionDetection(softShapeA, softShapeB, minDistance));
                
            }
                        
            return collisionPointStructure;
        }

        #endregion

    }
}

