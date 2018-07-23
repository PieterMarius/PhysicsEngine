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

using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.CollisionEngine.Dynamic_Bounding_Tree
{
    internal interface IAABB
    {
         AABB GetAABB();
    }

    internal class AABBNode : ICloneable
    {
        #region Fields

        public AABB aabb;
        public object Obj;

        public int? ParentNodeIndex;
        public int? LeftNodeIndex;
        public int? RightNodeIndex;
        public int? NextNodeIndex;

        #endregion

        #region Constructor

        #endregion

        #region Public Methods

        public bool IsLeaf()
        {
            return LeftNodeIndex == null;
        }

        public object Clone()
        {
            return new AABBNode()
            {
                aabb = new AABB(this.aabb.Min, this.aabb.Max, this.aabb.ObjectReference),
                Obj = this.Obj,
                ParentNodeIndex = this.ParentNodeIndex,
                LeftNodeIndex = this.LeftNodeIndex,
                RightNodeIndex = this.RightNodeIndex,
                NextNodeIndex = this.NextNodeIndex
            };
        }

        #endregion
    }

    internal sealed class AABBTree
    {
        #region Fields

        private List<AABBNode> Nodes;
        private SortedDictionary<IAABB, int> objectNodeIndexMap;

        private int? RootNodeIndex;
        private int AllocateNodeCount;
        private int? NextFreeNodeIndex;
        private int NodeCapacity;
        private int GrowthSize;

        #endregion

        #region Constructor

        public AABBTree()
        { }
                
        public AABBTree(int initialState)
        {
            Nodes = new List<AABBNode>();

            RootNodeIndex = null;
            AllocateNodeCount = 0;
            NextFreeNodeIndex = 0;
            NodeCapacity = initialState;
            GrowthSize = initialState;

            for (int nodeIndex = 0; nodeIndex < initialState; nodeIndex++)
            {
                Nodes.Add(new AABBNode());
                Nodes[nodeIndex].NextNodeIndex = nodeIndex + 1;
            }

            Nodes[initialState - 1].NextNodeIndex = null;
        }

        #endregion

        #region Public Methods

        //const
        public void InsertObject(IAABB obj)
        {
            int nodeIndex = AllocateNode();
            AABBNode node = Nodes[nodeIndex];
            node.aabb = obj.GetAABB();
            node.Obj = obj;
            InsertLeaf(nodeIndex);
            objectNodeIndexMap[obj] = nodeIndex;
        }
        
        //const
        public void RemoveObject(IAABB obj)
        {
            int nodeIndex = objectNodeIndexMap[obj];
            RemoveLeaf(nodeIndex);
            DeallocateNode(nodeIndex);
            objectNodeIndexMap.Remove(obj);
        }

        //const
        public void UpdateObject(IAABB obj)
        {
            int nodeIndex = objectNodeIndexMap[obj];
            UpdateLeaf(nodeIndex, obj.GetAABB());
        }
        
        //obj const and list
        public List<IAABB> QueryOverlaps(IAABB obj)
        {
            List<IAABB> overlaps = new List<IAABB>();
            Stack<int?> stack = new Stack<int?>();
            AABB testAabb = obj.GetAABB();

            stack.Push(RootNodeIndex);
            while(stack.Any())
            {
                int? nodeIndex = stack.Peek();
                stack.Pop();

                if (!nodeIndex.HasValue)
                    continue;

                //const
                AABBNode node = Nodes[nodeIndex.Value].Clone() as AABBNode;
                if (node.aabb.Intersect(testAabb))
                {
                    if(node.IsLeaf() && node.Obj != obj)
                    {
                        overlaps.Add((IAABB)node.Obj);
                    }
                    else
                    {
                        stack.Push(node.LeftNodeIndex);
                        stack.Push(node.RightNodeIndex);
                    }
                }
            }

            return overlaps;
        }
        
        #endregion

        #region Private Methods
                
        private int AllocateNode()
        {
            if(NextFreeNodeIndex == null)
            {
                NodeCapacity += GrowthSize;

                for (int nodeIndex = AllocateNodeCount; nodeIndex < NodeCapacity; nodeIndex++)
                {
                    Nodes.Add(new AABBNode());
                    Nodes[nodeIndex].NextNodeIndex = nodeIndex + 1;
                }

                Nodes[NodeCapacity - 1].NextNodeIndex = null;
                NextFreeNodeIndex = AllocateNodeCount;
            }

            int nodeIndexv = NextFreeNodeIndex.Value;
            AABBNode allocatedNode = Nodes[nodeIndexv];
            allocatedNode.ParentNodeIndex = null;
            allocatedNode.LeftNodeIndex = null;
            allocatedNode.RightNodeIndex = null;
            NextFreeNodeIndex = allocatedNode.NextNodeIndex;
            AllocateNodeCount++;
            
            return nodeIndexv;
        }

        private void DeallocateNode(int nodeIndex)
        {
            AABBNode deallocatedNode = Nodes[nodeIndex];
            deallocatedNode.NextNodeIndex = NextFreeNodeIndex;
            NextFreeNodeIndex = nodeIndex;
            AllocateNodeCount--;
        }

        private void InsertLeaf(int leafNodeIndex)
        {
            // make sure we're inserting a new leaf
            //        assert(_nodes[leafNodeIndex].parentNodeIndex == AABB_NULL_NODE);
            //        assert(_nodes[leafNodeIndex].leftNodeIndex == AABB_NULL_NODE);
            //        assert(_nodes[leafNodeIndex].rightNodeIndex == AABB_NULL_NODE);
            if(RootNodeIndex== null)
            {
                RootNodeIndex = leafNodeIndex;
                return;
            }

            int treeNodeIndex = RootNodeIndex.Value;
            AABBNode leafNode = Nodes[leafNodeIndex];

            while (!Nodes[treeNodeIndex].IsLeaf())
            {
                //const
                AABBNode treeNode = Nodes[treeNodeIndex].Clone() as AABBNode;
                int leftNodeIndex = treeNode.LeftNodeIndex.Value;
                int rightNodeIndex = treeNode.RightNodeIndex.Value;
                //const
                AABBNode leftNode = Nodes[leftNodeIndex].Clone() as AABBNode;
                //const
                AABBNode rightNode = Nodes[rightNodeIndex].Clone() as AABBNode;

                AABB combinedAabb = treeNode.aabb.Merge(leafNode.aabb);

                double newParentNodeCost = 2.0 * combinedAabb.SurfaceArea;
                double minimumPushDownCost = 2.0 * (combinedAabb.SurfaceArea - treeNode.aabb.SurfaceArea);

                double costLeft;
                double costRight;

                if(leftNode.IsLeaf())
                {
                    costLeft = leafNode.aabb.Merge(leftNode.aabb).SurfaceArea + minimumPushDownCost;
                }
                else
                {
                    AABB newLeftAabb = leafNode.aabb.Merge(leftNode.aabb);
                    costLeft = (newLeftAabb.SurfaceArea - leftNode.aabb.SurfaceArea) + minimumPushDownCost;
                }

                if (rightNode.IsLeaf())
                {
                    costRight = leafNode.aabb.Merge(rightNode.aabb).SurfaceArea + minimumPushDownCost;
                }
                else
                {
                    AABB newRightAabb = leafNode.aabb.Merge(rightNode.aabb);
                    costRight = (newRightAabb.SurfaceArea - rightNode.aabb.SurfaceArea) + minimumPushDownCost;
                }

                if (newParentNodeCost < costRight)
                    treeNodeIndex = leftNodeIndex;
                else
                    treeNodeIndex = rightNodeIndex;

            }

            int leafSiblingIndex = treeNodeIndex;
            AABBNode leafSibling = Nodes[leafSiblingIndex];
            int? oldParentIndex = leafSibling.ParentNodeIndex;
            int newParentIndex = AllocateNode();
            AABBNode newParent = Nodes[newParentIndex];
            newParent.ParentNodeIndex = oldParentIndex;
            newParent.aabb = leafNode.aabb.Merge(leafSibling.aabb);
            newParent.LeftNodeIndex = leafSiblingIndex;
            newParent.RightNodeIndex = leafNodeIndex;
            leafNode.ParentNodeIndex = newParentIndex;
            leafSibling.ParentNodeIndex = newParentIndex;

            if (oldParentIndex == null)
                RootNodeIndex = newParentIndex;
            else
            {
                AABBNode oldParent = Nodes[oldParentIndex.Value];
                if (oldParent.LeftNodeIndex == leafSiblingIndex)
                    oldParent.LeftNodeIndex = newParentIndex;
                else
                    oldParent.RightNodeIndex = newParentIndex;
            }

            treeNodeIndex = leafNode.ParentNodeIndex.Value;
            FixUpwardsTree(treeNodeIndex);
        }

        private void RemoveLeaf(int leafNodeIndex)
        {
            if (leafNodeIndex == RootNodeIndex)
            {
                RootNodeIndex = null;
                return;
            }

            AABBNode leafNode = Nodes[leafNodeIndex];
            int parentNodeIndex = leafNode.ParentNodeIndex.Value;
            //const
            AABBNode parentNode = Nodes[parentNodeIndex].Clone() as AABBNode;
            int? grandParentNodeIndex = parentNode.ParentNodeIndex;
            int? siblingNodeIndex = parentNode.LeftNodeIndex == leafNodeIndex ? parentNode.RightNodeIndex : parentNode.LeftNodeIndex;
            //assert(siblingNodeIndex != AABB_NULL_NODE); // we must have a sibling
            AABBNode siblingNode = Nodes[siblingNodeIndex.Value];

            if (grandParentNodeIndex != null)
            {
                AABBNode grandParentNode = Nodes[grandParentNodeIndex.Value];
                if (grandParentNode.LeftNodeIndex == parentNodeIndex)
                    grandParentNode.LeftNodeIndex = siblingNodeIndex;
                else
                    grandParentNode.RightNodeIndex = siblingNodeIndex;

                siblingNode.ParentNodeIndex = grandParentNodeIndex;
                DeallocateNode(parentNodeIndex);

                FixUpwardsTree(grandParentNodeIndex.Value);
            }
            else
            {
                RootNodeIndex = siblingNodeIndex;
                siblingNode.ParentNodeIndex = null;
                DeallocateNode(parentNodeIndex);
            }
            leafNode.ParentNodeIndex = null;
        }

        //const newAABB
        private void UpdateLeaf(int leafNodeIndex, AABB newAABB)
        {
            AABBNode node = Nodes[leafNodeIndex];

            if (node.aabb.Contains(newAABB))
                return;

            RemoveLeaf(leafNodeIndex);
            node.aabb = newAABB;
            InsertLeaf(leafNodeIndex);
        }

        private void FixUpwardsTree(int? treeNodeIndex)
        {
            while(treeNodeIndex != null)
            {
                AABBNode treeNode = Nodes[treeNodeIndex.Value];
                //assert(treeNode.leftNodeIndex != AABB_NULL_NODE && treeNode.rightNodeIndex != AABB_NULL_NODE);

                //const
                AABBNode leftNode = Nodes[treeNode.LeftNodeIndex.Value].Clone() as AABBNode;
                //const
                AABBNode rightNode = Nodes[treeNode.RightNodeIndex.Value].Clone() as AABBNode;
                treeNode.aabb = leftNode.aabb.Merge(rightNode.aabb);

                treeNodeIndex = treeNode.ParentNodeIndex;
            }
        }

        #endregion
    }
}
