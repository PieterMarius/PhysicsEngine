using System;
using PhysicsEngineMathUtility;

namespace ObjectDefinition
{
	public class CollisionPoint
	{
		public readonly Vector3 collisionPointA;
		public readonly Vector3 collisionPointB;
		public readonly Vector3 collisionNormal;

		#region "Constructors"

		public CollisionPoint()
		{
		}

		public CollisionPoint (
			Vector3 collisionPointA,
			Vector3 collisionPointB,
			Vector3 collisionNormal)
		{
			this.collisionPointA = collisionPointA;
			this.collisionPointB = collisionPointB;
			this.collisionNormal = collisionNormal;
		}

		#endregion

	}
}

