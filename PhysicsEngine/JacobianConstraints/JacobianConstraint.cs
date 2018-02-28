/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
	internal struct JacobianConstraint
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

        #region Constructorv

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

