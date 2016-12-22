using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public class CollisionDetectionEngine: ICollisionEngine
	{
		#region Private Fields

		GJK collisionEngine;
		EPA compenetrationCollisionEngine;
		readonly CollisionEngineParameters collisionEngineParameters;
		readonly SweepAndPruneEngine sweepAndPruneEngine;

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
			SimulationObject[] objects,
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
            SimulationObject A,
            SimulationObject B,
            int indexA,
            int indexB,
            int geometryIndexA,
            int geometryIndexB,
            double minDistance)
        {
            
                if (!gjkOutput.Intersection &&
                    gjkOutput.CollisionDistance <= minDistance)
                {
                    var mpg = new ManifoldPointsGenerator(
                                                  collisionEngineParameters.ManifoldPointNumber,
                                                  collisionEngineParameters.GJKManifoldTolerance,
                                                  collisionEngineParameters.ManifoldProjectionTolerance);

                    if (gjkOutput.CollisionNormal.Length() < 1E-15)
                        return null;

                    List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
                                                                   A,
                                                                   B,
                                                                   gjkOutput.CollisionPoint);

                    return new CollisionPointStructure(
                        indexA,
                        indexB,
                        gjkOutput.Intersection,
                        gjkOutput.CollisionDistance,
                        gjkOutput.CollisionPoint,
                        collisionPointsList.ToArray());
                }

            if (gjkOutput.Intersection)
            {
                EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
                                                A,
                                                B,
                                                geometryIndexA,
                                                geometryIndexB,
                                                gjkOutput.SupportTriangles,
                                                gjkOutput.Centroid);

                if (epaOutput.CollisionPoint.CollisionNormal.Length() < 1E-15)
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
                    gjkOutput.Intersection,
                    epaOutput.CompenetrationDistance,
                    epaOutput.CollisionPoint,
                    collisionPointsList.ToArray());

            }

            return null;
        }

		private CollisionPointStructure NarrowPhase(
			SimulationObject A,
			SimulationObject B,
			int indexA,
			int indexB,
			double minDistance)
		{
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            for (int i = 0; i < A.ObjectGeometry.Length; i++)
            {
                for (int j = 0; j < B.ObjectGeometry.Length; j++)
                {
                    GJKOutput gjkOutput = (collisionEngine.Execute(A, B, i, j));
                    collisionPointStructure.Add(NarrowPhaseCollisionControl(
                        gjkOutput, 
                        A, 
                        B, 
                        indexA, 
                        indexB,
                        i,
                        j,
                        minDistance));
                }
            }
            
            if(collisionPointStructure.Count > 1)
            {

            }

            return collisionPointStructure[0];
            
		}

		private List<CollisionPointStructure> BruteForceBroadPhase(
			SimulationObject[] objects,
			double minDistance)
		{
			var result = new List<CollisionPointStructure> ();

			var lockMe = new object();

			Parallel.For (0, 
				objects.Length, 
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber }, 
				i => {
					if (objects [i] != null) {
						for (int j = i + 1; j < objects.Length; j++) {
							if (objects [j] != null) {
								CollisionPointStructure collisionPointStruct = NarrowPhase (
									                                              objects [i], 
									                                              objects [j],
									                                              i,
									                                              j,
									                                              minDistance);

								lock (lockMe) {    
									if (collisionPointStruct != null)
										result.Add (collisionPointStruct);
								}
							}
						}
					}
				});
			
			return result;
		}

		private List<CollisionPointStructure> SweepAndPruneBroadPhase(
			SimulationObject[] objects,
			double minDistance)
		{
			var result = new List<CollisionPointStructure> ();

            AABB[][] boxs = Array.ConvertAll(objects, item => (item.ObjectGeometry == null) ? null : Array.ConvertAll(item.ObjectGeometry, x => x.AABBox));

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

		#endregion

	}
}

