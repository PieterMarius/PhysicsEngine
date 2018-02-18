using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.CollisionEngine;
using System.Linq;

namespace SharpPhysicsEngine
{
	internal class ContactPartitioningEngine: IContactPartitioningEngine
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
						cps.ObjectIndexA,
						cps.ObjectIndexB,
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

                Dictionary<int, ObjectType> objectsTypeDic = simulationObjects.ToDictionary(x => x.ID, x => x.ObjectType);
                
                while (contactIndex.Count != 0) {
                    var partition = new List<ContactIndex>
                    {
                        contactIndex[0]
                    };
                    RecursiveSearch (
						contactIndex [0],
						contactIndex,
						partition,
                        objectsTypeDic);

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

		private void RecursiveSearch(
			ContactIndex searchPoint,
			List<ContactIndex> readList,
			List<ContactIndex> partition,
            Dictionary<int, ObjectType> objectsTypeDic)
		{
			for (int i = 0; i < readList.Count; i++) 
			{
				ContactIndex collisionValue = readList [i];

				if (searchPoint.IndexA == collisionValue.IndexA &&
					searchPoint.IndexB == collisionValue.IndexB &&
				    searchPoint.KeyIndex == collisionValue.KeyIndex)
					continue;

				if (objectsTypeDic[searchPoint.IndexA] == ObjectType.StaticBody &&
                    objectsTypeDic[searchPoint.IndexB] == ObjectType.StaticBody)
					break;

				if (objectsTypeDic[searchPoint.IndexA] == ObjectType.StaticBody &&
				           (searchPoint.IndexB == collisionValue.IndexA ||
				           searchPoint.IndexB == collisionValue.IndexB)) {

					if (!partition.Contains (collisionValue)) 
					{
						partition.Add (collisionValue);

						RecursiveSearch (
							collisionValue,
							readList, 
							partition,
                            objectsTypeDic);
					}

				} else if (objectsTypeDic[searchPoint.IndexB] == ObjectType.StaticBody &&
				           (searchPoint.IndexA == collisionValue.IndexA ||
				           searchPoint.IndexA == collisionValue.IndexB)) {

					if (!partition.Contains (collisionValue)) 
					{
						partition.Add (collisionValue);

						RecursiveSearch (
							collisionValue,
							readList, 
							partition,
                            objectsTypeDic);
					}

				} else if ((objectsTypeDic[searchPoint.IndexA] == ObjectType.RigidBody &&
                           objectsTypeDic[searchPoint.IndexB] == ObjectType.RigidBody) &&
						   (searchPoint.IndexA == collisionValue.IndexB ||
				           searchPoint.IndexB == collisionValue.IndexA ||
				           searchPoint.IndexA == collisionValue.IndexA ||
				           searchPoint.IndexB == collisionValue.IndexB )) 
				{
					if (!partition.Contains (collisionValue)) 
					{
						partition.Add (collisionValue);

						RecursiveSearch (
							collisionValue,
							readList, 
							partition,
                            objectsTypeDic);
					}
				}
			}	
		}

		#endregion
	}
}

