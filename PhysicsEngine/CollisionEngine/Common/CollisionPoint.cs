using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	public struct CollisionPoint
	{
		public readonly VertexProperties CollisionPointA;
		public readonly VertexProperties CollisionPointB;
		public readonly Vector3 CollisionNormal;
		public List<StartImpulseProperties> StartImpulseValue { get; private set; }

		#region "Constructors"

		public CollisionPoint (
			VertexProperties collisionPointA,
			VertexProperties collisionPointB,
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

