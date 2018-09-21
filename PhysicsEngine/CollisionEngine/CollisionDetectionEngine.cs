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
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;
using static SharpPhysicsEngine.Helper.PhysicsEngineConst;
using SharpPhysicsEngine.CollisionEngine.Dynamic_Bounding_Tree;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal class CollisionDetectionEngine: ICollisionEngine
	{
		#region Private Fields
        						
		private readonly CollisionEngineParameters collisionEngineParameters;
        private INarrowPhase narrowPhase;
        private IBroadPhase broadPhaseEngine;
        private readonly double CollisionDistance;

		#endregion

		#region Constructor

		public CollisionDetectionEngine (
			CollisionEngineParameters collisionEngineParameters,
            double collisionDistance)
		{
			this.collisionEngineParameters = collisionEngineParameters;
            
            narrowPhase = new NarrowPhase(collisionEngineParameters);
            SetBroadPhaseEngine();

			CollisionDistance = collisionDistance;
            
        }

		#endregion

		#region Public Methods

		#region Interface ICollisionEngine

		/// <summary>
		/// Runs the test collision.
		/// </summary>
		/// <returns>The test collision.</returns>
		/// <param name="shapes">Objects.</param>
		/// <param name="minDistance">Minimum distance.</param>
		public List<CollisionPointStructure> Execute(IShape[] shapes)
		{
           return ExecuteEngine(shapes); 
		}


        /// <summary>
        /// Test collision between collision pair
        /// </summary>
        /// <param name="shapes"></param>
        /// <param name="collisionPair"></param>
        /// <param name="collisionDistance"></param>
        /// <returns></returns>
        public List<CollisionPointStructure> Execute(
            IShape[] shapes,
            List<CollisionPair> collisionPair,
            double collisionDistance)
        {
            return ExecuteEngine(shapes, collisionPair, collisionDistance);
        }

        public void AddShape(IShape newShape)
        {
            broadPhaseEngine.AddShape(newShape);
        }

        public void RemoveShape(IShape oldShape)
        {
            broadPhaseEngine.RemoveShape(oldShape);
        }

        public void UpdateShape(IShape shape)
        {
            broadPhaseEngine.UpdateShape(shape);
        }

        public List<AABBNode> GetHierarchicalTree()
        {
            if (broadPhaseEngine is HierarchicalTree ht)
                return ht.GetNodes();

            return null;
        }

        #endregion

        #endregion

        #region Private Methods

        private void SetBroadPhaseEngine()
        {
            switch(collisionEngineParameters.BroadPhaseType)
            {
                case BroadPhaseEngineType.AABBBroadPhase:
                    broadPhaseEngine = new AABBBroadPhase(collisionEngineParameters);
                    break;

                case BroadPhaseEngineType.HierarchicalTree:
                    broadPhaseEngine = new HierarchicalTree(collisionEngineParameters);
                    break;

                case BroadPhaseEngineType.BruteForce:
                default:
                    broadPhaseEngine = new BruteForceBroadPhase(collisionEngineParameters);
                    break;
            }
        }
        				
		private List<CollisionPointStructure> ExecuteEngine(IShape[] shapes)
		{
            List<CollisionPair> collisionPair = broadPhaseEngine.Execute (shapes, CollisionDistance);

            var result = narrowPhase.Execute(shapes, collisionPair, collisionEngineParameters.CollisionDistance);

			return result;
		}

        private List<CollisionPointStructure> ExecuteEngine(
            IShape[] shapes,
            List<CollisionPair> collisionPair,
            double collisionDistance)
        {
            return narrowPhase.Execute(shapes, collisionPair, collisionDistance);
        }

        #endregion

    }
}

