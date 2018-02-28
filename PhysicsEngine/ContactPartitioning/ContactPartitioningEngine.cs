/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

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

