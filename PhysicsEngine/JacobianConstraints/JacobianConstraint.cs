using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
	public struct JacobianConstraint
	{
		#region Public Properties

		/// <summary>
		/// The object A.
		/// </summary>
		public readonly IShapeCommon ObjectA;

		/// <summary>
		/// The object B.
		/// </summary>
		public readonly IShapeCommon ObjectB;

		/// <summary>
		/// The contact reference.
		/// </summary>
		public int? ContactReference;

		/// <summary>
		/// The linear component object A.
		/// </summary>
		public readonly Vector3? LinearComponentA;

		/// <summary>
		/// The linear component object B.
		/// </summary>
		public readonly Vector3? LinearComponentB;

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
		/// The correction value.
		/// </summary>
		public readonly double CorrectionValue;

		/// <summary>
		/// The constraint limit.
		/// </summary>
		public readonly double ConstraintLimit;

		/// <summary>
		/// The CFM (Constraint Force Mixing).
		/// </summary>
		public readonly double CFM;

		/// <summary>
		/// The index of the collision struct.
		/// </summary>
		public readonly StartImpulseProperties StartImpulse;
        
        #endregion

        #region Constructor

        public JacobianConstraint (
            IShapeCommon objectA,
            IShapeCommon objectB,
			int? contactReference,
			Vector3? linearComponentA,
			Vector3? linearComponentB,
			Vector3 angularComponentA,
			Vector3 angularComponentB,
			ConstraintType type,
			double B,
			double correctionValue,
			double cfm,
			double constraintLimit,
			StartImpulseProperties startImpulse)
		{
			ObjectA = objectA;
			ObjectB = objectB;
			ContactReference = contactReference;
			LinearComponentA = linearComponentA;
			LinearComponentB = linearComponentB;
			AngularComponentA = angularComponentA;
			AngularComponentB = angularComponentB;
			Type = type;
			this.B = B;
			CorrectionValue = correctionValue;
			CFM = cfm;
			ConstraintLimit = constraintLimit;
			StartImpulse = startImpulse;
		}
               
        public JacobianConstraint(
            IShapeCommon objectA,
            IShapeCommon objectB,
            Vector3 linearComponentA,
            Vector3 linearComponentB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            ConstraintType type,
            double B,
            double correctionValue,
            double cfm,
            double constraintLimit)
            : this(objectA, objectB, null, linearComponentA, linearComponentB, angularComponentA, angularComponentB, type, B, correctionValue, cfm, constraintLimit, new StartImpulseProperties(0.0))
        { }

        public JacobianConstraint(
            IShapeCommon objectA,
            IShapeCommon objectB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            ConstraintType type,
            double B,
            double correctionValue,
            double cfm,
            double constraintLimit)
            : this(objectA, objectB, null, null, null, angularComponentA, angularComponentB, type, B, correctionValue, cfm, constraintLimit, new StartImpulseProperties(0.0))
        { }

        #endregion

        #region Public Methods

        public void SetContactReference(int? contactRef)
        {
            ContactReference = contactRef;
        }

        #endregion
    }
}

