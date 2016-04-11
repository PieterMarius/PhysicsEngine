using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public class SweepAndPruneEngine
	{
		private CollisionEngineParameters collisionEngineParameters;

		public SweepAndPruneEngine(CollisionEngineParameters collisionEngineParameters)
		{
			this.collisionEngineParameters = collisionEngineParameters;
		}

		#region Public Methods

		public List<CollisionPair> SweepAndPruneTest(AABB[] boxs)
		{
			List<CollisionPair> collisionPairs = new List<CollisionPair> ();

			object lockMe = new object();

			Parallel.For (0, 
				boxs.Length, 
				new ParallelOptions { MaxDegreeOfParallelism = this.collisionEngineParameters.MaxThreadNumber }, 
				i => {
					for (int j = i + 1; j < boxs.Length; j++) {

						if (testAABBAABB (boxs [i], boxs [j], 0) &&
							testAABBAABB (boxs [i], boxs [j], 1) &&
							testAABBAABB (boxs [i], boxs [j], 2)) 
						{
							lock (lockMe) { 
								collisionPairs.Add (new CollisionPair (i, j));
							}
						}
					}
				});

			return collisionPairs;
		}

		#endregion

		#region Private Methods

		private bool testAABBAABB(
			AABB a, 
			AABB b,
			int axisIndex)
		{
			return a.Min [axisIndex] <= b.Max [axisIndex] &&
				a.Max [axisIndex] >= b.Min [axisIndex];
		}

		#endregion
	}
}

