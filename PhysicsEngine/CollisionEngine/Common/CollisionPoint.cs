using System.Collections.Generic;
using PhysicsEngineMathUtility;
using ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	public struct CollisionPoint
	{
		public readonly Vector3 CollisionPointA;
		public readonly Vector3 CollisionPointB;
		public readonly Vector3 CollisionNormal;
		public List<StartImpulseProperties> StartImpulseValue { get; private set; }

		#region "Constructors"

		public CollisionPoint (
			Vector3 collisionPointA,
			Vector3 collisionPointB,
			Vector3 collisionNormal)
		{
			CollisionPointA = collisionPointA;
			CollisionPointB = collisionPointB;
            CollisionNormal = collisionNormal;

			//Start Impulse Proprties respectively of Normal, Friction Axis1 and Friction Axis2
			StartImpulseValue = new List<StartImpulseProperties>()
			{
				new StartImpulseProperties(0.0),
				new StartImpulseProperties(0.0),
				new StartImpulseProperties(0.0),
			};
		}

		#endregion

	}
}

