using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;

namespace SharpPhysicsEngine.CollisionEngine
{
	public struct CollisionPoint
	{
		public readonly VertexProperties CollisionPointA;
		public readonly VertexProperties CollisionPointB;
		public readonly Vector3 CollisionNormal;
		public List<StartImpulseProperties> StartImpulseValue { get; private set; }

		#region Constructor

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

        #region Public Methods

        public VertexProperties GetCollisionVertex(int index)
        {
            switch (index)
            {
                case 0:
                    return CollisionPointA;
                case 1:
                    return CollisionPointB;
                default:
                    throw new ArgumentException("Incorrect collision point");
            }
        }

        #endregion Public Methods

    }
}

