using System;
using PhysicsEngineMathUtility;
using LCPSolver;

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
		/// The contact reference.
		/// </summary>
		public readonly int ContactReference;

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
		/// The suggested start value for solver solution find.
		/// </summary>
		public readonly double Solution;

		#endregion

		#region Constructor

		public JacobianContact (
			int objectA,
			int objectB,
			int contactReference,
			Vector3 linearComponentA,
			Vector3 linearComponentB,
			Vector3 angularComponentA,
			Vector3 angularComponentB,
			ConstraintType type,
			double b,
			double constraintLimit,
			double solution)
		{
			this.ObjectA = objectA;
			this.ObjectB = objectB;
			this.ContactReference = contactReference;
			this.LinearComponentA = linearComponentA;
			this.LinearComponentB = linearComponentB;
			this.AngularComponentA = angularComponentA;
			this.AngularComponentB = angularComponentB;
			this.Type = type;
			this.B = b;
			this.ConstraintLimit = constraintLimit;
			this.Solution = solution;
		}

		#endregion
	}
}

