using SimulationObjectDefinition;
using System.Collections.Generic;
using CollisionEngine;

namespace MonoPhysicsEngine
{
	public class ContactPartitioningEngine: IContactPartitioningEngine
	{

		#region Public Methods

		public List<SpatialPartition> calculateSpatialPartitioning(
			List<CollisionPointStructure> collisionPoints,
			List<IConstraint> simulationJoint,
			SimulationObject[] simulationObjects)
		{
			if (collisionPoints.Count > 0 ||
				simulationJoint.Count > 0) 
			{
				var partitions = new List<SpatialPartition> ();

				var contactIndex = new List<ContactIndex>();

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
						simulationJoint [i].GetObjectIndexA(),
						simulationJoint [i].GetObjectIndexB(),
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

					for (int i = 0; i < partition.Count; i++) 
					{
						var index = new List<int> ();
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

