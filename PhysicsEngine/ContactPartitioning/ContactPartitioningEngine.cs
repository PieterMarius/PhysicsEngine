using SimulationObjectDefinition;
using System.Collections.Generic;
using CollisionEngine;

namespace MonoPhysicsEngine
{
	public class ContactPartitioningEngine: IContactPartitioningEngine
	{

		#region Public Methods

		public List<SpatialPartition> calculateSpatialPartitioning(
			CollisionPointStructure[] collisionPoints,
			List<IConstraint> simulationJoints,
			SimulationObject[] simulationObjects)
		{
			if (collisionPoints.Length > 0 ||
				simulationJoints.Count > 0) 
			{
				var partitions = new List<SpatialPartition> ();

				var contactIndex = new List<ContactIndex> ();

				// Add contacts
				foreach (CollisionPointStructure cps in collisionPoints)
				{
					contactIndex.Add (new ContactIndex (
						cps.ObjectA,
						cps.ObjectB,
						ContactGroupType.Collision));
				}

				// Add joints
				foreach (IConstraint smj in simulationJoints)
				{
					contactIndex.Add (new ContactIndex (
						smj.GetObjectIndexA(),
						smj.GetObjectIndexB(),
						ContactGroupType.Joint));
				}

				while (contactIndex.Count != 0) {
					var partition = new List<ContactIndex> ();

					partition.Add (contactIndex [0]);
					recursiveSearch (
						contactIndex [0],
						contactIndex,
						partition,
						simulationObjects);

					foreach(ContactIndex cIndex in partition)
					{
						var index = new List<int> ();
						int indexVealue = 0;

						foreach(ContactIndex cntIndex in contactIndex)
						{
							
							if (cntIndex.IndexA == cIndex.IndexA &&
								cntIndex.IndexB == cIndex.IndexB &&
								cntIndex.Type == cIndex.Type) 
							{
								index.Add (indexVealue);
							}
							indexVealue++;
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
			for (int i = 0; i < readList.Count; i++) 
			{
				ContactIndex collisionValue = readList [i];

				if (collisionPoint.IndexA == collisionValue.IndexA &&
					collisionPoint.IndexB == collisionValue.IndexB)
					continue;

				if (simulationObjects [collisionPoint.IndexA].ObjectType == ObjectType.StaticRigidBody &&
				    simulationObjects [collisionPoint.IndexB].ObjectType == ObjectType.StaticRigidBody)
					break;

				if (simulationObjects [collisionPoint.IndexA].ObjectType == ObjectType.StaticRigidBody &&
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

				} else if (simulationObjects [collisionPoint.IndexB].ObjectType == ObjectType.StaticRigidBody &&
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
						   (simulationObjects [collisionPoint.IndexA].ObjectType == ObjectType.RigidBody &&
						   simulationObjects [collisionPoint.IndexB].ObjectType == ObjectType.RigidBody)) {

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

