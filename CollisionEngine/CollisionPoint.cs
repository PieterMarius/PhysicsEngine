using PhysicsEngineMathUtility;

namespace CollisionEngine
{
	public class CollisionPoint
	{
		public readonly Vector3 CollisionPointA;
		public readonly Vector3 CollisionPointB;
		public readonly Vector3 CollisionNormal;
		public double StartImpulseValue { get; private set; }

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
			CollisionPointA = collisionPointA;
			CollisionPointB = collisionPointB;
			CollisionNormal = collisionNormal;
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

		#region Public Methods

		public void SetStartImpulseValue(double startImpulse)
		{
			StartImpulseValue = startImpulse;
		}

		#endregion

	}
}

