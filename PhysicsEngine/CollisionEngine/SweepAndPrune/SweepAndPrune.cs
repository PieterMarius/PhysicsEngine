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
using System.Threading.Tasks;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal class SweepAndPruneEngine
	{
		#region Fileds

		CollisionEngineParameters collisionEngineParameters;

		#endregion

		#region Contructor

		public SweepAndPruneEngine(CollisionEngineParameters collisionEngineParameters)
		{
			this.collisionEngineParameters = collisionEngineParameters;
		}

		#endregion

		#region Public Methods

		public List<CollisionPair> Execute(
			AABB[][] boxs,
			double distanceTolerance)
		{
			var collisionPairs = new List<CollisionPair> ();

			var lockMe = new object();

			Parallel.For (0, 
				boxs.Length, 
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber }, 
				i => {

					for (int k = 0; k < boxs[i].Length; k++)
					{
						AABB box1 = boxs[i][k];

						for (int j = i + 1; j < boxs.Length; j++)
						{
							for (int w = 0; w < boxs[j].Length; w++)
							{
								AABB box2 = boxs[j][w];

								if (boxs[i] != null && boxs[j] != null &&
									TestBoxes(box1, box2, 0, distanceTolerance) &&
									TestBoxes(box1, box2, 1, distanceTolerance) &&
									TestBoxes(box1, box2, 2, distanceTolerance))
								{
									lock (lockMe)
									{
										collisionPairs.Add(new CollisionPair(i, j));
									}
								}
							}
						}
					}
					
				});

			return collisionPairs;
		}

		#endregion

		#region Private Methods

		private bool TestBoxes(
			AABB a, 
			AABB b,
			int axisIndex,
			double distanceTolerance)
		{
			return a.Min [axisIndex] - b.Max[axisIndex] <= distanceTolerance &&
				   a.Max [axisIndex] - b.Min[axisIndex] >= -distanceTolerance ;
		}

		#endregion
	}
}

