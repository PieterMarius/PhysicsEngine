using System.Collections.Generic;
using System.Threading.Tasks;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public class SweepAndPruneEngine
	{
		#region Fileds

		CollisionEngineParameters collisionEngineParameters;

		#endregion

		#region Contructor

		public SweepAndPruneEngine(CollisionEngineParameters collisionEngineParameters)
		{
			this.collisionEngineParameters = collisionEngineParameters;
		}

		#endregion

		#region Public Methods

		public List<CollisionPair> Execute(
			AABB[] boxs,
			double distanceTolerance)
		{
			var collisionPairs = new List<CollisionPair> ();

			var lockMe = new object();

			Parallel.For (0, 
				boxs.Length, 
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber }, 
				i => {
					AABB box1 = boxs[i];
					
					for (int j = i + 1; j < boxs.Length; j++) {
						AABB box2 = boxs[j];
						
						if (boxs[i] != null && boxs[j] != null &&
							testAABBAABB (box1, box2, 0, distanceTolerance) &&
							testAABBAABB (box1, box2, 1, distanceTolerance) &&
							testAABBAABB (box1, box2, 2, distanceTolerance)) 
						{
							lock (lockMe) 
							{ 
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
			int axisIndex,
			double distanceTolerance)
		{
			return a.Min [axisIndex] - b.Max[axisIndex] <= distanceTolerance &&
				   a.Max [axisIndex] - b.Min[axisIndex] >= -distanceTolerance ;
		}

		#endregion
	}
}

