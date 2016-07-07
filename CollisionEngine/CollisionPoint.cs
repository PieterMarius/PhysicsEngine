using PhysicsEngineMathUtility;

namespace CollisionEngine
{
	public class CollisionPoint
	{
		public readonly Vector3 CollisionPointA;
		public readonly Vector3 CollisionPointB;
		public readonly Vector3 CollisionNormal;
		public double StartImpulseValue;

		#region "Constructors"

		public CollisionPoint()
		{
		}

		public CollisionPoint (
			Vector3 collisionPointA,
			Vector3 collisionPointB,
			Vector3 collisionNormal,
			double startImpulseValue)
		{
			this.CollisionPointA = collisionPointA;
			this.CollisionPointB = collisionPointB;
			this.CollisionNormal = collisionNormal;
			StartImpulseValue = startImpulseValue;
		}

		public CollisionPoint (
			Vector3 collisionPointA,
			Vector3 collisionPointB,
			Vector3 collisionNormal)
			:this(collisionPointA, collisionPointB, collisionNormal, 0.0)
		{
		}

		#endregion

	}
}

