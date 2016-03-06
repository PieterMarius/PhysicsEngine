using System;
using SimulationObjectDefinition;
using System.Collections.Generic;
using CollisionEngine;

namespace MonoPhysicsEngine
{
	public class ContactPartitioningEngine: IContactPartitioningEngine
	{

		#region Constructor

		public ContactPartitioningEngine ()
		{}

		#endregion

		#region Public Methods

		public List<SpatialPartition> calculateSpatialPartitioning(
			List<CollisionPointStructure> collisionPoints,
			List<SimulationJoint> simulationJoint,
			SimulationObject[] simulationObjects)
		{
			if (collisionPoints.Count > 0 ||
				simulationJoint.Count > 0) 
			{
				List<SpatialPartition> partitions = new List<SpatialPartition> ();

				List<ContactIndex> contactIndex = new List<ContactIndex>();

				// Add contacts
				for (int i = 0; i < collisionPoints.Count; i++) 
				{
					contactIndex.Add (new ContactIndex (
						collisionPoints [i].ObjectA,
						collisionPoints [i].ObjectB,
						ContactGroupType.Collision));
				}

				// Add joints
				for (int i = 0; i < simulationJoint.Count; i++) 
				{
					contactIndex.Add (new ContactIndex (
						simulationJoint [i].IndexA,
						simulationJoint [i].IndexB,
						ContactGroupType.Joint));
				}

				while (contactIndex.Count != 0) {
					List<ContactIndex> partition = new List<ContactIndex> ();

					partition.Add (contactIndex [0]);
					recursiveSearch (
						contactIndex [0],
						contactIndex,
						partition,
						simulationObjects);

					for (int i = 0; i < partition.Count; i++) 
					{
						List<int> index = new List<int> ();
						for (int j = 0; j < contactIndex.Count; j++) 
						{
							if (contactIndex [j].IndexA == partition [i].IndexA &&
								contactIndex [j].IndexB == partition [i].IndexB &&
								contactIndex [j].Type == partition [i].Type) 
							{
								index.Add (j);
							}
						}
						for (int j = 0; j < index.Count; j++) 
						{
							contactIndex.RemoveAt (index [j]);
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
			ContactIndex collisionPoint,
			List<ContactIndex> readList,
			List<ContactIndex> partition,
			SimulationObject[] simulationObjects)
		{
			for (int i = 0; i < readList.Count; i++) {
				ContactIndex collisionValue = readList [i];

				if (collisionPoint.IndexA == collisionValue.IndexA &&
				    collisionPoint.IndexB == collisionValue.IndexB) {
					continue;
				} else if (simulationObjects [collisionPoint.IndexA].Mass <= 0.0 &&
				           simulationObjects [collisionPoint.IndexB].Mass <= 0.0) {
					break;
				} else if (simulationObjects [collisionPoint.IndexA].Mass <= 0.0 &&
				           (collisionPoint.IndexB == collisionValue.IndexA ||
				           collisionPoint.IndexB == collisionValue.IndexB)) {

					if (!partition.Contains (collisionValue)) {
						partition.Add (collisionValue);
						recursiveSearch (
							collisionValue, 
							readList, 
							partition,
							simulationObjects);
					}

				} else if (simulationObjects [collisionPoint.IndexB].Mass <= 0.0 &&
				           (collisionPoint.IndexA == collisionValue.IndexA ||
				           collisionPoint.IndexA == collisionValue.IndexB)) {

					if (!partition.Contains (collisionValue)) {
						partition.Add (collisionValue);
						recursiveSearch (
							collisionValue, 
							readList, 
							partition,
							simulationObjects);
					}

				} else if (collisionPoint.IndexA == collisionValue.IndexB ||
				           collisionPoint.IndexB == collisionValue.IndexA ||
				           collisionPoint.IndexA == collisionValue.IndexA ||
				           collisionPoint.IndexB == collisionValue.IndexB &&
				           (simulationObjects [collisionPoint.IndexA].Mass > 0.0 &&
				           simulationObjects [collisionPoint.IndexB].Mass > 0.0)) {

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

