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
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal class AABBBroadPhase: IBroadPhase
	{
		#region Fields

		CollisionEngineParameters collisionEngineParameters;

		#endregion

		#region Contructor

		public AABBBroadPhase(CollisionEngineParameters collisionEngineParameters)
		{
			this.collisionEngineParameters = collisionEngineParameters;
		}

		#endregion

		#region Public Methods

		public List<CollisionPair> Execute(
			AABB[] boxs,
			double distanceTolerance)
		{
			var collisionPairs = new List<CollisionPair> ();

            var rangePartitioner = Partitioner.Create(0, boxs.Length, Convert.ToInt32(boxs.Length / collisionEngineParameters.MaxThreadNumber) + 1);

            var lockMe = new object();

			Parallel.ForEach (
                rangePartitioner, 
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
                (range, loopState) => 
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        AABB box1 = boxs[i];

                        for (int k = i + 1; k < boxs.Length; k++)
                        {
                            AABB box2 = boxs[k];

                            if (box1 != null && box2 != null &&
                                Helper.TestBoxes(box1, box2, 0, distanceTolerance) &&
                                Helper.TestBoxes(box1, box2, 1, distanceTolerance) &&
                                Helper.TestBoxes(box1, box2, 2, distanceTolerance))
                            {
                                lock (lockMe)
                                {
                                    collisionPairs.Add(new CollisionPair(i, k));
                                }
                            }
                        }
                    }
					
				});

			return collisionPairs;
		}

        public List<CollisionPair> Execute(
            AABB[] objectBoxesA,
            AABB[] objectBoxesB,
            double distanceTolerance)
        {
            var collisionPairs = new List<CollisionPair>();

            var rangePartitioner = Partitioner.Create(0, objectBoxesA.Length, Convert.ToInt32(objectBoxesA.Length / collisionEngineParameters.MaxThreadNumber) + 1);

            var lockMe = new object();

            Parallel.ForEach(
                rangePartitioner,
                new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
                (range, loopState) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        AABB box1 = objectBoxesA[i];

                        for (int k = 0; k < objectBoxesB.Length; k++)
                        {
                            AABB box2 = objectBoxesB[k];

                            if (box1 != null && box2 != null &&
                                Helper.TestBoxes(box1, box2, 0, distanceTolerance) &&
                                Helper.TestBoxes(box1, box2, 1, distanceTolerance) &&
                                Helper.TestBoxes(box1, box2, 2, distanceTolerance))
                            {
                                lock (lockMe)
                                {
                                    collisionPairs.Add(new CollisionPair(i, k));
                                }
                            }
                        }
                    }
                });

            return collisionPairs;
        }

        public Vector3d Execute(AABB boxA, AABB boxB)
        {
            return AABB.GetDist(boxA, boxB);
        }

        #endregion

        #region Private Methods
               
		#endregion
	}
}

