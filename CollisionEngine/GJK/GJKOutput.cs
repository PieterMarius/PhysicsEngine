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
			CollisionDistance = collisionDistance;
			CollisionPoint = collisionPoint;
			CollisionNormal = collisionNormal;
			Intersection = intersection;
			MinSimplex = minSimplex;
		}

		#endregion
	}
}

