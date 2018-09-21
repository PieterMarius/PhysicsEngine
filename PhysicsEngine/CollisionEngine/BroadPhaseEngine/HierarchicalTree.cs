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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.CollisionEngine.Dynamic_Bounding_Tree;
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.ShapeDefinition;
using static SharpPhysicsEngine.Helper.CommonUtilities;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class HierarchicalTree : IBroadPhase
    {
        #region Fields

        private readonly CollisionEngineParameters collisionEngineParameters;
        private readonly AABBTree hierarchicalTree;
        
        #endregion

        #region Contructor

        public HierarchicalTree(
            CollisionEngineParameters collisionEngineParameters)
        {
            this.collisionEngineParameters = collisionEngineParameters;
            this.hierarchicalTree = new AABBTree(1);
        }

        #endregion

        #region Public Methods

        public List<CollisionPair> Execute(
            IShape[] shapes,
            double distanceTolerance)
        {
            var result = new HashSet<CollisionPair>();
            var IDs = new Dictionary<int, int>();

            for (int i = 0; i < shapes.Length; i++)
                IDs.Add(shapes[i].ID, i);
            
            for (int i = 0; i < shapes.Length; i++)
            {
                var overlaps = hierarchicalTree.QueryOverlaps(ExtractIAABBFromShape(shapes[i]));
                foreach (var item in overlaps)
                {
                    var aabb = item.GetAABB();
                    int ID = ((IShape)aabb.ObjectReference).ID;
                    var cp = new CollisionPair(i, IDs[ID]);
                    result.Add(cp);
                }
            }

            return result.ToList();
        }

        public Vector3d Execute(AABB boxA, AABB boxB)
        {
            throw new NotImplementedException();
        }

        public void RemoveShape(IShape shape)
        {
            hierarchicalTree.RemoveObject(ExtractIAABBFromShape(shape));
        }

        public void AddShape(IShape shape)
        {
            hierarchicalTree.InsertObject(ExtractIAABBFromShape(shape));
        }

        public void UpdateShape(IShape shape)
        {
            hierarchicalTree.UpdateObject(ExtractIAABBFromShape(shape));
        }

        public List<AABBNode> GetNodes()
        {
            return hierarchicalTree.GetNodes();
        }

        #endregion

        #region Private Methods
                
        private void UpdateHierarchicalTree()
        {

        }

        #endregion
    }
}
