using System;
using ObjectDefinition;
using System.Collections.Generic;

namespace MonoPhysicsEngine
{
	public class ContactPartitioningEngine: IContactPartitioningEngine
	{

		#region Constructor

		public ContactPartitioningEngine ()
		{
		}

		#endregion

		#region Public Methods

		public List<SpatialPartition> calculateSpatialPartitioning(
			List<CollisionPointStructure> collisionPoints,
			SimulationObject[] simulationObjects)
		{
			if (collisionPoints.Count > 0) 
			{
				List<SpatialPartition> partitions = new List<SpatialPartition> ();

				//DEBUG creo copia lista
				List<CollisionPointStructure> collisionPointsCopy = new List<CollisionPointStructure>();
				for (int i = 0; i < collisionPoints.Count; i++) 
				{
					collisionPointsCopy.Add (collisionPoints [i]);
				}

				while (collisionPointsCopy.Count != 0) {
					List<CollisionPointStructure> partition = new List<CollisionPointStructure> ();

					partition.Add (collisionPointsCopy [0]);
					recursiveSearch (
						collisionPointsCopy [0],
						collisionPointsCopy,
						partition,
						simulationObjects);

					Console.WriteLine ("NPartition contact " + partition.Count);

					for (int i = 0; i < partition.Count; i++) 
					{
						List<int> index = new List<int> ();
						for (int j = 0; j < collisionPointsCopy.Count; j++) 
						{
							if (collisionPointsCopy [j].ObjectA == partition [i].ObjectA &&
								collisionPointsCopy [j].ObjectB == partition [i].ObjectB) 
							{
								index.Add (j);
							}
						}
						for (int j = 0; j < index.Count; j++) 
						{
							collisionPointsCopy.RemoveAt (index [j]);
						}

					}

					partitions.Add (new SpatialPartition (partition));
				}
				return partitions;
			}
			return null;
		}

		#endregion

		#region Private mehtods

		private void recursiveSearch(
			CollisionPointStructure collisionPoint,
			List<CollisionPointStructure> readList,
			List<CollisionPointStructure> partition,
			SimulationObject[] simulationObjects)
		{
			for (int i = 0; i < readList.Count; i++) {
				CollisionPointStructure collisionValue = readList [i];

				if (collisionPoint.ObjectA == collisionValue.ObjectA &&
				    collisionPoint.ObjectB == collisionValue.ObjectB) {
					continue;
				} else if (simulationObjects [collisionPoint.ObjectA].Mass <= 0.0 &&
				           simulationObjects [collisionPoint.ObjectB].Mass <= 0.0) {
					break;
				} else if (simulationObjects [collisionPoint.ObjectA].Mass <= 0.0 &&
				           (collisionPoint.ObjectB == collisionValue.ObjectA ||
				           collisionPoint.ObjectB == collisionValue.ObjectB)) {

					if (!partition.Contains (collisionValue)) {
						partition.Add (collisionValue);
						recursiveSearch (
							collisionValue, 
							readList, 
							partition,
							simulationObjects);
					}

				} else if (simulationObjects [collisionPoint.ObjectB].Mass <= 0.0 &&
				           (collisionPoint.ObjectA == collisionValue.ObjectA ||
				           collisionPoint.ObjectA == collisionValue.ObjectB)) {

					if (!partition.Contains (collisionValue)) {
						partition.Add (collisionValue);
						recursiveSearch (
							collisionValue, 
							readList, 
							partition,
							simulationObjects);
					}

				} else if (collisionPoint.ObjectA == collisionValue.ObjectB ||
				             collisionPoint.ObjectB == collisionValue.ObjectA ||
				             collisionPoint.ObjectA == collisionValue.ObjectA ||
				             collisionPoint.ObjectB == collisionValue.ObjectB &&
				             (simulationObjects [collisionPoint.ObjectA].Mass > 0.0 &&
				             simulationObjects [collisionPoint.ObjectB].Mass > 0.0)) {

					if (!partition.Contains (collisionValue)) {
						partition.Add (collisionValue);
						recursiveSearch (
							collisionValue, 
							readList, 
							partition,
							simulationObjects);
					}
				}
			}	
		}

		#endregion
	}
}

