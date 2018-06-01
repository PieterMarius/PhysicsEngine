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

namespace SharpPhysicsEngine.CollisionEngine
{
	public class CollisionEngineParameters
	{
		#region Collision Engine Parameters Properties
	
		/// <summary>
		/// Gets the max GJK iteration.
		/// </summary>
		/// <value>The max GJK iteration.</value>
		public int MaxGJKIteration { get; private set; }

		/// <summary>
		/// Max iteration number for searching the deepest compenetrations point.
		/// </summary>
		/// <value>The max EPA iteration.</value>
		public int MaxEPAIteration { get; private set; }

		/// <summary>
		/// Gets the stabilization parameter for collision engine.
		/// </summary>
		/// <value>The precision.</value>
		public double Precision { get; private set; }

		/// <summary>
		/// Gets the GJK Manifold tolerance.
		/// </summary>
		/// <value>The GJKM anifold tolerance.</value>
		public double GJKManifoldTolerance { get; private set;}

		/// <summary>
		/// Gets the EPA Manifold tolerance parameter.
		/// </summary>
		/// <value>The EPAM anifold tolerance.</value>
		public double EPAManifoldTolerance { get; private set;}

		/// <summary>
		/// Gets the manifold projection tolerance.
		/// </summary>
		/// <value>The manifold projection tolerance.</value>
		public double ManifoldProjectionTolerance { get; private set; }

		/// <summary>
		/// Gets the manifold points number.
		/// </summary>
		/// <value>The manifold point number.</value>
		public int ManifoldPointNumber { get; private set;}

		/// <summary>
		/// Gets the max thread number.
		/// </summary>
		/// <value>The max thread number.</value>
		public int MaxThreadNumber { get; private set;}

		/// <summary>
		/// Gets a value indicating whether this CollisionEngineParameters activate sweep and prune.
		/// </summary>
		/// <value><c>true</c> if activate sweep and prune; otherwise, <c>false</c>.</value>
		public BroadPhaseEngineType BroadPhaseType { get; private set;}

        /// <summary>
        /// Collision distance
        /// </summary>
        public double CollisionDistance { get; private set; }

        #endregion

        #region Constructors

        public CollisionEngineParameters ()
		{
			MaxGJKIteration = 12;
			MaxEPAIteration = 12;
			Precision = 0.0000001;
			GJKManifoldTolerance = 0.009;
			EPAManifoldTolerance = 0.009;
			ManifoldProjectionTolerance = 0.005;
			ManifoldPointNumber = 5;
			MaxThreadNumber = 2;
            CollisionDistance = 0.001;
			BroadPhaseType = BroadPhaseEngineType.AABBBroadPhase;
		}

		public CollisionEngineParameters (
			double collisionDistance,
            int maxGJKIteration,
			int maxEPAIteration,
			double precision,
			double gjkTolerance,
			double epaTolerance,
			double manifoldProjectionTolerance,
			int manifoldPointNumber,
			int maxThreadNumber,
            BroadPhaseEngineType broadPhaseEngineType)
		{
            CollisionDistance = collisionDistance;
			MaxGJKIteration = maxGJKIteration;
			MaxEPAIteration = maxEPAIteration;
			Precision = precision;
			GJKManifoldTolerance = gjkTolerance;
			EPAManifoldTolerance = epaTolerance;
			ManifoldProjectionTolerance = manifoldProjectionTolerance;
			ManifoldPointNumber = manifoldPointNumber;
			MaxThreadNumber = maxThreadNumber;
			BroadPhaseType = broadPhaseEngineType;
		}

		#endregion

		#region Public Methods

		public void SetGJKMaxIteration(int maxIteration)
		{
			MaxGJKIteration = maxIteration;
		}

		public void SetEPAMaxIteration(int maxIteration)
		{
			MaxEPAIteration = maxIteration;
		}
			
		public void SetPrecision(double precision)
		{
			Precision = precision;
		}

        public void SetManifoldPoints(int manifoldPointNumber)
		{
			ManifoldPointNumber = manifoldPointNumber;
		}

        public void SetCollisionDistance(double collisionDistance)
        {
            CollisionDistance = collisionDistance;
        }
			
		#endregion
	}
}

