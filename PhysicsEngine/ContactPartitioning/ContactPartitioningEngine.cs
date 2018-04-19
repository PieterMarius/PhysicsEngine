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
using SharpPhysicsEngine.ContactPartitioning;
using System;

namespace SharpPhysicsEngine
{
	internal class ContactPartitioningEngine: IContactPartitioningEngine
	{

        #region Public Methods

        public List<Partition> GetPartitions(
            CollisionPointStructure[] collisionPoints,
            List<IConstraint> joints,
            IShape[] shapes,
            ISoftShape[] softShapes)
        {
            List<SpatialPartition> spatialPartitions = CalculateSpatialPartitioning(collisionPoints, joints, shapes);

            List<Partition> partitions = new List<Partition>();

            if (spatialPartitions != null)
            {
                HashSet<int> totObjectIndex = new HashSet<int>();

                for (int i = 0; i < spatialPartitions.Count; i++)
                {
                    Partition partitionItem = new Partition();
                    HashSet<int> objectIndex = new HashSet<int>();

                    for (int j = 0; j < spatialPartitions[i].ObjectList.Count; j++)
                    {
                        if (spatialPartitions[i].ObjectList[j].Type == ContactGroupType.Collision)
                        {
                            CollisionPointStructure cpStruct = ConstraintHelper.Find(
                                collisionPoints,
                                spatialPartitions[i].ObjectList[j]);

                            if (cpStruct != null)
                                partitionItem.PartitionedCollisionPoints.Add(cpStruct);

                            objectIndex.Add(cpStruct.ObjectIndexA);
                            objectIndex.Add(cpStruct.ObjectIndexB);
                            totObjectIndex.Add(cpStruct.ObjectIndexA);
                            totObjectIndex.Add(cpStruct.ObjectIndexB);

                        }
                        else
                        {
                            IConstraint smJoint = joints.Find(item =>
                                                  item.GetObjectIndexA() == spatialPartitions[i].ObjectList[j].IndexA &&
                                                  item.GetObjectIndexB() == spatialPartitions[i].ObjectList[j].IndexB &&
                                                  item.GetKeyIndex() == spatialPartitions[i].ObjectList[j].KeyIndex);

                            partitionItem.PartitionedJoints.Add(smJoint);

                            objectIndex.Add(smJoint.GetObjectIndexA());
                            objectIndex.Add(smJoint.GetObjectIndexB());
                            totObjectIndex.Add(smJoint.GetObjectIndexA());
                            totObjectIndex.Add(smJoint.GetObjectIndexB());

                        }
                    }

                    ////Add Soft Body Constraints if is involved in collision
                    foreach (var item in objectIndex)
                    {
                        if (shapes[item] is ISoftShape softShape)
                            partitionItem.PartitionedJoints.AddRange(softShape.SoftConstraint);
                    }

                    partitions.Add(partitionItem);
                }

                ////Add Soft Body Constraints if is not involved in collision
                foreach (var item in softShapes)
                {
                    if (!totObjectIndex.Contains(((IShape)item).ID))
                    {
                        Partition partitionItem = new Partition();
                        partitionItem.PartitionedJoints.AddRange(item.SoftConstraint);
                        partitions.Add(partitionItem);
                    }
                }
            }
            else if (softShapes.Length > 0)
            {
                foreach (var softShape in softShapes)
                {
                    Partition partitionItem = new Partition();
                    partitionItem.PartitionedJoints.AddRange(softShape.SoftConstraint);
                    partitions.Add(partitionItem);
                }
            }

            return partitions;
        }

        #endregion

        #region Private mehtods

        private List<SpatialPartition> CalculateSpatialPartitioning(
            CollisionPointStructure[] collisionPoints,
            List<IConstraint> simulationJoints,
            IShape[] simulationObjects)
        {
            if (collisionPoints.Length > 0 ||
                simulationJoints.Count > 0)
            {
                var partitions = new List<SpatialPartition>();

                var contactIndex = new List<ContactIndex>();

                // Add contacts
                int keyIndex = 0;
                foreach (CollisionPointStructure cps in collisionPoints)
                {
                    contactIndex.Add(new ContactIndex(
                        cps.ObjectIndexA,
                        cps.ObjectIndexB,
                        ContactGroupType.Collision,
                        keyIndex));

                    keyIndex++;
                }

                // Add joints
                foreach (IConstraint smj in simulationJoints)
                {
                    contactIndex.Add(new ContactIndex(
                        smj.GetObjectIndexA(),
                        smj.GetObjectIndexB(),
                        ContactGroupType.Joint,
                        smj.GetKeyIndex()));
                }

                Dictionary<int, Tuple<ObjectType, bool>> objectsTypeDic = simulationObjects.ToDictionary(
                    x => x.ID, 
                    x => new Tuple<ObjectType, bool>(x.ObjectType, x.IsStatic));

                while (contactIndex.Count != 0)
                {
                    var partition = new List<ContactIndex>
                    {
                        contactIndex[0]
                    };

                    RecursiveSearch(
                        contactIndex[0],
                        contactIndex,
                        partition,
                        objectsTypeDic);

                    foreach (ContactIndex cIndex in partition)
                    {
                        var index = new List<int>();
                        int indexVealue = 0;

                        foreach (ContactIndex cntIndex in contactIndex)
                        {

                            if (cntIndex.IndexA == cIndex.IndexA &&
                                cntIndex.IndexB == cIndex.IndexB &&
                                cntIndex.Type == cIndex.Type)
                            {
                                index.Add(indexVealue);
                            }
                            indexVealue++;
                        }

                        index.Sort((a, b) => -1 * a.CompareTo(b));
                        for (int j = 0; j < index.Count; j++)
                        {
                            contactIndex.RemoveAt(index[j]);
                        }
                    }

                    partitions.Add(new SpatialPartition(partition));
                }
                return partitions;
            }
            return null;
        }

        private void RecursiveSearch(
			ContactIndex searchPoint,
			List<ContactIndex> readList,
			List<ContactIndex> partition,
            Dictionary<int, Tuple<ObjectType, bool>> objectsTypeDic)
		{
			for (int i = 0; i < readList.Count; i++) 
			{
				ContactIndex collisionValue = readList [i];

				if (searchPoint.IndexA == collisionValue.IndexA &&
					searchPoint.IndexB == collisionValue.IndexB &&
				    searchPoint.KeyIndex == collisionValue.KeyIndex)
					continue;

				if (objectsTypeDic[searchPoint.IndexA].Item2 &&
                    objectsTypeDic[searchPoint.IndexB].Item2)
					break;

				if (objectsTypeDic[searchPoint.IndexA].Item2 &&
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

				} else if (objectsTypeDic[searchPoint.IndexB].Item2 &&
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

				} else if ((objectsTypeDic[searchPoint.IndexA].Item1 == ObjectType.RigidBody &&
                           objectsTypeDic[searchPoint.IndexB].Item1 == ObjectType.RigidBody) &&
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

