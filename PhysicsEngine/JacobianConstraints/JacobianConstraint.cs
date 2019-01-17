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
        /// The contact reference (used by friction contact)
        /// </summary>
        public int? ContactReference;

		/// <summary>
		/// The linear component object A.
		/// </summary>
		public readonly Vector3d? LinearComponentA;

		/// <summary>
		/// The linear component object B.
		/// </summary>
		public readonly Vector3d? LinearComponentB;

		/// <summary>
		/// The angular component object A.
		/// </summary>
		public readonly Vector3d AngularComponentA;

		/// <summary>
		/// The angular component object B.
		/// </summary>
		public readonly Vector3d AngularComponentB;

		/// <summary>
		/// The type of contact (collision, friction, joint).
		/// </summary>
		public readonly ConstraintType Type;

		/// <summary>
		/// The expeceted known value.
		/// </summary>
		public readonly double JacobianVelocity;

		/// <summary>
		/// The correction value.
		/// </summary>
		public readonly double CorrectionValue;

		/// <summary>
		/// The constraint limit.
		/// </summary>
		public readonly double ConstraintLimit;

        /// <summary>
        /// The Constraint expected value
        /// </summary>
        public readonly double ConstraintValue;

		/// <summary>
		/// The CFM (Constraint Force Mixing).
		/// </summary>
		public readonly double CFM;
        		
        #endregion

        #region Constructorv

        public JacobianConstraint (
            IShapeCommon objectA,
            IShapeCommon objectB,
			int? contactReference,
			Vector3d? linearComponentA,
			Vector3d? linearComponentB,
			Vector3d angularComponentA,
			Vector3d angularComponentB,
			ConstraintType type,
            double constraintValue,
			double jacobianVelocity,
			double correctionValue,
			double cfm,
			double constraintLimit)
		{
			ObjectA = objectA;
			ObjectB = objectB;
			ContactReference = contactReference;
			LinearComponentA = linearComponentA;
			LinearComponentB = linearComponentB;
			AngularComponentA = angularComponentA;
			AngularComponentB = angularComponentB;
			Type = type;
            ConstraintValue = constraintValue;
			JacobianVelocity = jacobianVelocity;
			CorrectionValue = correctionValue;
			CFM = cfm;
			ConstraintLimit = constraintLimit;
		}
               
        public JacobianConstraint(
            IShapeCommon objectA,
            IShapeCommon objectB,
            Vector3d linearComponentA,
            Vector3d linearComponentB,
            Vector3d angularComponentA,
            Vector3d angularComponentB,
            ConstraintType type,
            double constraintValue,
            double jacobianVelocity,
            double correctionValue,
            double cfm,
            double constraintLimit)
            : this(objectA, objectB, null, linearComponentA, linearComponentB, angularComponentA, angularComponentB, type, constraintValue, jacobianVelocity, correctionValue, cfm, constraintLimit)
        { }

        public JacobianConstraint(
            IShapeCommon objectA,
            IShapeCommon objectB,
            Vector3d angularComponentA,
            Vector3d angularComponentB,
            ConstraintType type,
            double constraintValue,
            double jacobianVelocity,
            double correctionValue,
            double cfm,
            double constraintLimit)
            : this(objectA, objectB, null, null, null, angularComponentA, angularComponentB, type, constraintValue, jacobianVelocity, correctionValue, cfm, constraintLimit)
        { }

        public JacobianConstraint(JacobianConstraint jc)
        {
            ObjectA = jc.ObjectA;
            ObjectB = jc.ObjectB;
            ContactReference = jc.ContactReference;
            LinearComponentA = jc.LinearComponentA;
            LinearComponentB = jc.LinearComponentB;
            AngularComponentA = jc.AngularComponentA;
            AngularComponentB = jc.AngularComponentB;
            Type = jc.Type;
            JacobianVelocity = jc.JacobianVelocity;
            ConstraintValue = jc.ConstraintValue;
            JacobianVelocity = jc.JacobianVelocity;
            CorrectionValue = jc.CorrectionValue;
            CFM = jc.CFM;
            ConstraintLimit = jc.ConstraintLimit;
        }

        #endregion

        #region Public Methods

        public void SetContactReference(int? contactRef)
        {
            ContactReference = contactRef;
        }

        #endregion
    }
}

