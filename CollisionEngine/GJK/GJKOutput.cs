using System;
using ObjectDefinition;
using PhysicsEngineMathUtility;

namespace CollisionEngine
{
	public class GJKOutput
	{
		#region Public Fields

		public readonly double CollisionDistance;
		public readonly CollisionPoint CollisionPoint;
		public readonly Vector3 CollisionNormal;
		public readonly bool Intersection;
		public readonly Simplex MinSimplex;

		#endregion

		#region Constructor

		public GJKOutput (
			double collisionDistance,
			CollisionPoint collisionPoint,
			Vector3 collisionNormal,
			bool intersection,
			Simplex minSimplex)
		{
			this.CollisionDistance = collisionDistance;
			this.CollisionPoint = collisionPoint;
			this.CollisionNormal = collisionNormal;
			this.Intersection = intersection;
			this.MinSimplex = minSimplex;
		}

		#endregion
	}
}

