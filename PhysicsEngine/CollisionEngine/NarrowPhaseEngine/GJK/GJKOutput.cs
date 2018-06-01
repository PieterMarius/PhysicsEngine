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
using SharpEngineMathUtility;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal class GJKOutput
	{
		#region Public Fields

		public readonly double CollisionDistance;
		public readonly CollisionPoint CollisionPoint;
		public readonly Vector3 CollisionNormal;
		public readonly Vector3 Centroid;
		public readonly bool Intersection;
		public readonly List<SupportTriangle> SupportTriangles;

		#endregion

		#region Constructor

		public GJKOutput (
			double collisionDistance,
			CollisionPoint collisionPoint,
			Vector3 collisionNormal,
			Vector3 centroid,
			bool intersection,
			List<SupportTriangle> supportTriangles)
		{
			CollisionDistance = collisionDistance;
			CollisionPoint = collisionPoint;
			CollisionNormal = collisionNormal;
			Centroid = centroid;
			Intersection = intersection;
			SupportTriangles = supportTriangles;
		}

		#endregion
	}
}

