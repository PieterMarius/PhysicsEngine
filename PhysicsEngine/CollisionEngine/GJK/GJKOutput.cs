using System.Collections.Generic;
using PhysicsEngineMathUtility;

namespace CollisionEngine
{
	public class GJKOutput
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

