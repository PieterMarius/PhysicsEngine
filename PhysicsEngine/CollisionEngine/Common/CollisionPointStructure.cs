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
using System.Linq;

namespace SharpPhysicsEngine.CollisionEngine
{
	public sealed class CollisionPointStructure
	{
		#region Public Properties

		/// <summary>
		/// The object a.
		/// </summary>
		public readonly int ObjectIndexA;

		/// <summary>
		/// The object b.
		/// </summary>
		public readonly int ObjectIndexB;

		/// <summary>
		/// Collision Points Base Structure
		/// </summary>
		public CollisionPointBaseStructure[] CollisionPointBase { get; private set; }

		/// <summary>
		/// Persistent contact time counter
		/// </summary>
		public int FrameCount { get; private set; }

		#endregion

		#region Constructors

		public CollisionPointStructure (
			int objectIndexA,
			int objectIndexB,
			CollisionPointBaseStructure[] collisionPointBase)
		{
			ObjectIndexA = objectIndexA;
			ObjectIndexB = objectIndexB;
			CollisionPointBase = collisionPointBase;
			FrameCount = 0;
		}

		public CollisionPointStructure(
			int objectA,
			int objectB,
			CollisionPointBaseStructure collisionPointBase)
			: this(objectA, objectB, new CollisionPointBaseStructure[] { collisionPointBase })
		{ }

		#endregion

		#region Public Methods

		public void SetFrameCount(int count)
		{
			FrameCount = count;
		}

		public void SetBaseCollisionPoint(
			CollisionPointBaseStructure[] collisionPointBase)
		{
			CollisionPointBase = collisionPointBase;
		}

		#endregion

	}

	public class CollisionPointBaseStructure
	{
		#region Fields

		/// <summary>
		/// The collision point.
		/// </summary>
		public CollisionPoint CollisionPoint;

		/// <summary>
		/// The collision points.
		/// </summary>
		public CollisionPoint[] CollisionPoints;

		#endregion

		#region Constructor

		public CollisionPointBaseStructure(
			CollisionPoint collisionPoint,
			CollisionPoint[] collisionPoints)
		{
			CollisionPoint = collisionPoint;
			CollisionPoints = collisionPoints;
		}

		#endregion

		#region Public Methods

        public void AddCollisionPoint(CollisionPoint point)
        {
            var lst = CollisionPoints.ToList();
            lst.Add(point);
            CollisionPoints = lst.ToArray();
        }

		#endregion

	}
}

