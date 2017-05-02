using System.Collections.Generic;
using ShapeDefinition;
using CollisionEngine;

namespace SharpPhysicsEngine
{
	public class ContactPartitioningEngine: IContactPartitioningEngine
	{

		#region Public Methods

		public List<SpatialPartition> CalculateSpatialPartitioning(
			CollisionPointStructure[] collisionPoints,
			List<IConstraint> simulationJoints,
			IShape[] simulationObjects)
		{
			if (collisionPoints.Length > 0 ||
				simulationJoints.Count > 0) 
			{
				var partitions = new List<SpatialPartition> ();

				var contactIndex = new List<ContactIndex> ();

				// Add contacts
				int keyIndex = 0;
				foreach (CollisionPointStructure cps in collisionPoints)
				{
					contactIndex.Add (new ContactIndex (
						cps.ObjectA,
						cps.ObjectB,
						ContactGroupType.Collision,
						keyIndex));

					keyIndex++;
				}

				// Add joints
				foreach (IConstraint smj in simulationJoints)
				{
					contactIndex.Add (new ContactIndex (
						smj.GetObjectIndexA(),
						smj.GetObjectIndexB(),
						ContactGroupType.Joint,
						smj.GetKeyIndex()));
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

						index.Sort((a, b) => -1 * a.CompareTo(b));
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
			ContactIndex searchPoint,
			List<ContactIndex> readList,
			List<ContactIndex> partition,
			IShape[] simulationObjects)
		{
			for (int i = 0; i < readList.Count; i++) 
			{
				ContactIndex collisionValue = readList [i];

				if (searchPoint.IndexA == collisionValue.IndexA &&
					searchPoint.IndexB == collisionValue.IndexB &&
				    searchPoint.KeyIndex == collisionValue.KeyIndex)
					continue;

				if (simulationObjects [searchPoint.IndexA].ObjectType == ObjectType.StaticRigidBody &&
				    simulationObjects [searchPoint.IndexB].ObjectType == ObjectType.StaticRigidBody)
					break;

				if (simulationObjects [searchPoint.IndexA].ObjectType == ObjectType.StaticRigidBody &&
				           (searchPoint.IndexB == collisionValue.IndexA ||
				           searchPoint.IndexB == collisionValue.IndexB)) {

					if (!partition.Contains (collisionValue)) 
					{
						partition.Add (collisionValue);

						recursiveSearch (
							collisionValue,
							readList, 
							partition,
							simulationObjects);
					}

				} else if (simulationObjects [searchPoint.IndexB].ObjectType == ObjectType.StaticRigidBody &&
				           (searchPoint.IndexA == collisionValue.IndexA ||
				           searchPoint.IndexA == collisionValue.IndexB)) {

					if (!partition.Contains (collisionValue)) 
					{
						partition.Add (collisionValue);

						recursiveSearch (
							collisionValue,
							readList, 
							partition,
							simulationObjects);
					}

				} else if ((simulationObjects[searchPoint.IndexA].ObjectType == ObjectType.RigidBody &&
						   simulationObjects[searchPoint.IndexB].ObjectType == ObjectType.RigidBody) &&
						   (searchPoint.IndexA == collisionValue.IndexB ||
				           searchPoint.IndexB == collisionValue.IndexA ||
				           searchPoint.IndexA == collisionValue.IndexA ||
				           searchPoint.IndexB == collisionValue.IndexB )) 
				{
					if (!partition.Contains (collisionValue)) 
					{
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

