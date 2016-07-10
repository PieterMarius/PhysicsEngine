using PhysicsEngineMathUtility;
using SimulationObjectDefinition;
using CollisionEngine;

namespace MonoPhysicsEngine
{
	public struct JacobianContact
	{
		#region Public Properties

		/// <summary>
		/// The object index A.
		/// </summary>
		public readonly int ObjectA;

		/// <summary>
		/// The object index B.
		/// </summary>
		public readonly int ObjectB;

		/// <summary>
		/// The index of the collision struct.
		/// </summary>
		public readonly CollisionPoint CollisionPointStr;

		/// <summary>
		/// The contact reference.
		/// </summary>
		public readonly int? ContactReference;

		/// <summary>
		/// The linear component object A.
		/// </summary>
		public readonly Vector3 LinearComponentA;

		/// <summary>
		/// The linear component object B.
		/// </summary>
		public readonly Vector3 LinearComponentB;

		/// <summary>
		/// The angular component object A.
		/// </summary>
		public readonly Vector3 AngularComponentA;

		/// <summary>
		/// The angular component object B.
		/// </summary>
		public readonly Vector3 AngularComponentB;

		/// <summary>
		/// The type of contact (collision, friction, joint).
		/// </summary>
		public readonly ConstraintType Type;

		/// <summary>
		/// The expeceted known value.
		/// </summary>
		public readonly double B;

		/// <summary>
		/// The constraint limit.
		/// </summary>
		public readonly double ConstraintLimit;

		/// <summary>
		/// The CFM (Constraint Force Mixing).
		/// </summary>
		public readonly double CFM;

		/// <summary>
		/// The suggested start value for solver solution find.
		/// </summary>
		public readonly double StartImpulseValue;

		#endregion

		#region Constructor

		public JacobianContact (
			int objectA,
			int objectB,
			int? contactReference,
			Vector3 linearComponentA,
			Vector3 linearComponentB,
			Vector3 angularComponentA,
			Vector3 angularComponentB,
			ConstraintType type,
			double b,
			double cfm,
			double constraintLimit,
			double startImpulseValue,
			CollisionPoint collisionPoint)
		{
			ObjectA = objectA;
			ObjectB = objectB;
			ContactReference = contactReference;
			LinearComponentA = linearComponentA;
			LinearComponentB = linearComponentB;
			AngularComponentA = angularComponentA;
			AngularComponentB = angularComponentB;
			Type = type;
			B = b;
			CFM = cfm;
			ConstraintLimit = constraintLimit;
			StartImpulseValue = startImpulseValue;
			CollisionPointStr = collisionPoint;
		}

		public JacobianContact (
			int objectA,
			int objectB,
			int? contactReference,
			Vector3 linearComponentA,
			Vector3 linearComponentB,
			Vector3 angularComponentA,
			Vector3 angularComponentB,
			ConstraintType type,
			double b,
			double cfm,
			double constraintLimit,
			double startImpulseValue)
			:this(objectA, objectB, contactReference, linearComponentA, linearComponentB,
				angularComponentA, angularComponentB, type, b, cfm, constraintLimit, startImpulseValue,
				null)
		{
		}


		#endregion
	}
}

