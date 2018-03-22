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

using System;
using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
    internal sealed class Hinge2Constraint: Constraint
	{
		#region Private Fields

		const JointType jointType = JointType.Hinge2;

        private IShape ExternalSyncShape;
        private double SpringCoefficientHingeAxis;
        private Vector3 StartAnchorPoint;
        private Vector3 HingeAxis;
        private Vector3 RotationAxis;
        private Vector3 StartErrorAxis1;
        private Vector3 StartErrorAxis2;
        private Quaternion RelativeOrientation1;
        private Quaternion RelativeOrientation2;

        private double? AngularLimitMin1;
        private double? AngularLimitMax1;
        private double? AngularLimitMin2;
        private double? AngularLimitMax2;

        private double? SpeedHingeAxisLimit;
        private double? ForceHingeAxisLimit;
        private double? SpeedRotationAxisLimit;
        private double? ForceRotationAxisLimit;
        private Vector3 AnchorPoint;

        #endregion

        #region Constructor

        public Hinge2Constraint(
            IShape shapeA,
            IShape shapeB,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double errorReductionParam,
            double springCoefficientHingeAxis,
            double springCoefficient)
            : this(shapeA, shapeB, null, startAnchorPosition, hingeAxis, rotationAxis, errorReductionParam, springCoefficientHingeAxis, springCoefficient)
        { }

        public Hinge2Constraint(
            IShape shapeA,
            IShape shapeB,
            IShape externalSyncShape,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double errorReductionParam,
            double springCoefficientHingeAxis,
            double springCoefficient)
            :base(shapeA,shapeB,errorReductionParam,springCoefficient)
        {
            ExternalSyncShape = externalSyncShape;
            SpringCoefficientHingeAxis = springCoefficientHingeAxis;
            StartAnchorPoint = startAnchorPosition;
            HingeAxis = hingeAxis.Normalize();
            RotationAxis = rotationAxis.Normalize();

            InitConstraint();
        }
                
        #endregion
        
        #region Public Methods

        #region IConstraintBuilder

        /// <summary>
        /// Builds the Universal joint.
        /// </summary>
        /// <returns>The Universal joint.</returns>
        /// <param name="simulationObjs">Simulation objects.</param>
        public override List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
		{
			var hinge2Constraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            			
			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 hingeAxis = simulationObjectA.RotationMatrix * HingeAxis;
			Vector3 rotationAxis = simulationObjectB.RotationMatrix * RotationAxis;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3 t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

            #endregion

            #region Jacobian Constraint

            double freq = 1.0 / timeStep;
            double errorReduction = ErrorReductionParam * freq;
            double springCoefficient = SpringCoefficient * freq;

            #region Base Constraint

            ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double constraintLimit = errorReduction * Vector3.Dot (t1,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

            //DOF 2

            constraintLimit = errorReduction * Vector3.Dot(tempPerpendicular, linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
                tempPerpendicular,
				-1.0 * tempPerpendicular,
				Vector3.Cross (r1, tempPerpendicular),
				-1.0 * Vector3.Cross (r2, tempPerpendicular),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 3

			ConstraintType hingeAxisConstraintType = ConstraintType.Joint;
			if (SpringCoefficientHingeAxis > 0)
				hingeAxisConstraintType = ConstraintType.SoftJoint;

			constraintLimit = errorReduction * Vector3.Dot (hingeAxis,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
                hingeAxis,
				-1.0 * hingeAxis,
				Vector3.Cross (r1, hingeAxis),
				-1.0 * Vector3.Cross (r2, hingeAxis),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficientHingeAxis,
				0.0,
				hingeAxisConstraintType));
			
			//DOF 4

			double angularLimit = errorReduction * (-k);

			hinge2Constraints.Add (
				JacobianCommon.GetDOF (
                    	t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
                    springCoefficient,
					0.0,
					constraintType));

            hinge2Constraints.AddRange(GetSyncConstraintsExternalShape(
                hingeAxis, 
                rotationAxis,
                errorReduction));
            
            #endregion

            #region Limit Constraints 

            hinge2Constraints.AddRange(GetAngularLimit(
				simulationObjectA,
				simulationObjectB,
				hingeAxis,
				rotationAxis,
                errorReduction));

			#endregion

			#region Motor Constraint

			hinge2Constraints.AddRange(GetMotorConstraint(
				simulationObjectA,
				simulationObjectB,
				hingeAxis,
				rotationAxis));

			#endregion

			#endregion

			return hinge2Constraints;
		}

		#endregion

		#region IConstraint

		public override JointType GetJointType()
		{
			return jointType;
		}

        	public override Vector3 GetAnchorPosition()
		{
			return (ShapeA.RotationMatrix *
                   (StartAnchorPoint - ShapeA.StartPosition)) +
                   ShapeA.Position;
        }
        		
		public override void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedHingeAxisLimit = speedValue;
			ForceHingeAxisLimit = forceLimit;
		}

		public override void SetAxis2Motor(double speedValue, double forceLimit)
		{
			SpeedRotationAxisLimit = speedValue;
			ForceRotationAxisLimit = forceLimit;
		}

		public override void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin1 = angularLimitMin;
			AngularLimitMax1 = angularLimitMax;
		}

		public override void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin2 = angularLimitMin;
			AngularLimitMax2 = angularLimitMax;
		}

		public override void AddTorque(double torqueAxis1, double torqueAxis2)
		{
            Vector3 hingeAxis = ShapeA.RotationMatrix * HingeAxis;
            Vector3 rotationAxis = ShapeB.RotationMatrix * RotationAxis;

            Vector3 torque = rotationAxis * torqueAxis2 + hingeAxis * torqueAxis1;

            ShapeA.SetTorque(ShapeA.TorqueValue + torque);
            ShapeB.SetTorque(ShapeB.TorqueValue - torque);
        }

        public void RotateAxis1(double angle)
        {
            Vector3 hingeAxis = ShapeA.RotationMatrix * HingeAxis;
            Vector3 rotationAxis = ShapeB.RotationMatrix * RotationAxis;

            var rotationQuaternion = new Quaternion(hingeAxis, angle);
            var rt = (rotationQuaternion * ShapeB.RotationStatus).Normalize();
            var bufRotationAxis = rt.ConvertToMatrix() * RotationAxis;

            double angle1 = GetAngle1(
                    hingeAxis,
                    bufRotationAxis,
                    HingeAxis,
                    ShapeA.RotationStatus,
                    RelativeOrientation1);

            if(angle1 > AngularLimitMin1.Value &&
               angle1 < AngularLimitMax1.Value)
            {
                ShapeB.Rotate(hingeAxis, angle);
            }
            else if (angle1 < AngularLimitMin1.Value)
            {
                angle1 = GetAngle1(
                    hingeAxis,
                    rotationAxis,
                    HingeAxis,
                    ShapeA.RotationStatus,
                    RelativeOrientation1);

                double anglediff = AngularLimitMin1.Value - angle1;
                ShapeB.Rotate(hingeAxis, anglediff);
            }
            else if (angle1 > AngularLimitMax1.Value)
            {
                angle1 = GetAngle1(
                    hingeAxis,
                    rotationAxis,
                    HingeAxis,
                    ShapeA.RotationStatus,
                    RelativeOrientation1);

                double anglediff = AngularLimitMax1.Value - angle1;
                ShapeB.Rotate(hingeAxis, anglediff);
            }            
        }
                
        #region NotImplementedMethods

        public override void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotSupportedException();
		}

        #endregion

        #endregion

        #endregion

        #region Private Methods

        private void InitConstraint()
        {
            Vector3 relativePos = StartAnchorPoint - ShapeA.StartPosition;
            relativePos = ShapeA.RotationMatrix * relativePos;

            AnchorPoint = relativePos + ShapeA.Position;

            StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeA.Position);

            StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeB.Position);

            Vector3 rHingeAxis = ShapeA.RotationMatrix * HingeAxis;
            Vector3 rRotationAxis = ShapeB.RotationMatrix * RotationAxis;

            RelativeOrientation1 = CalculateRelativeOrientation(
                rHingeAxis,
                rRotationAxis,
                ShapeA.RotationStatus);

            RelativeOrientation2 = CalculateRelativeOrientation(
                rRotationAxis,
                rHingeAxis,
                ShapeB.RotationStatus);
        }

        double GetAngle2(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			return -GetAngle1(axis2, axis1, startAxis, rotationStatus, startRelativeRotation);
		}

		double GetAngle1(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

			Quaternion mult1 = Quaternion.Multiply1(rotationStatus, rotationQ);
			Quaternion mult2 = Quaternion.Multiply2(mult1, startRelativeRotation);

			var quaternionVectorPart = new Vector3(
				mult2.b,
				mult2.c,
				mult2.d);

			return JacobianCommon.GetRotationAngle(quaternionVectorPart, mult2.a, startAxis);
		}

		Quaternion CalculateRelativeOrientation(
			Vector3 axis1,
			Vector3 axis2,
			Quaternion bodyRotationStatus)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

			return Quaternion.Multiply1(bodyRotationStatus, rotationQ);
		}

		List<JacobianConstraint> GetAngularLimit(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
            double errorReduction)
		{
			var angularConstraint = new List<JacobianConstraint>();

			if (AngularLimitMin1.HasValue &&
				AngularLimitMax1.HasValue)
			{
				double angle1 = GetAngle1(
					hingeAxis,
					rotationAxis,
					HingeAxis,
					simulationObjectA.RotationStatus,
					RelativeOrientation1);

				JacobianConstraint? jContact = 
					JacobianCommon.GetAngularLimit (
                        angle1,
                        errorReduction,
						0.0,
						simulationObjectA,
						simulationObjectB,
						hingeAxis,
						AngularLimitMin1.Value,
						AngularLimitMax1.Value);
				
				if (jContact != null)
					angularConstraint.Add (jContact.Value);
			}

			if (AngularLimitMin2.HasValue &&
				AngularLimitMax2.HasValue)
			{
				double angle2 = GetAngle2(
					hingeAxis,
					rotationAxis,
					RotationAxis,
					simulationObjectB.RotationStatus,
					RelativeOrientation2);

				JacobianConstraint? jContact = 
					JacobianCommon.GetAngularLimit (
                        angle2,
                        errorReduction,
						0.0,
						simulationObjectA,
						simulationObjectB,
						rotationAxis,
						AngularLimitMin2.Value,
						AngularLimitMax2.Value);

				if (jContact != null)
					angularConstraint.Add (jContact.Value);

			}

			return angularConstraint;
		}

		private List<JacobianConstraint> GetMotorConstraint(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3 hingeAxis,
			Vector3 rotationAxis)
		{
			var motorConstraint = new List<JacobianConstraint>();

			if (SpeedHingeAxisLimit.HasValue &&
				ForceHingeAxisLimit.HasValue)
			{
				motorConstraint.Add(
					JacobianCommon.GetDOF(
                        new Vector3(),
						new Vector3(),
						-1.0 * hingeAxis,
						1.0 * hingeAxis,
						simulationObjectA,
						simulationObjectB,
						SpeedHingeAxisLimit.Value,
						0.0,
						0.0,
						ForceHingeAxisLimit.Value,
						ConstraintType.JointMotor));
			}

			if (SpeedRotationAxisLimit.HasValue &&
				ForceRotationAxisLimit.HasValue)
			{
				motorConstraint.Add(
					JacobianCommon.GetDOF(
                        new Vector3(),
						new Vector3(),
						-1.0 * rotationAxis,
						1.0 * rotationAxis,
						simulationObjectA,
						simulationObjectB,
						SpeedRotationAxisLimit.Value,
						0.0,
						0.0,
						ForceRotationAxisLimit.Value,
						ConstraintType.JointMotor));
			}

			return motorConstraint;
		}

        private List<JacobianConstraint> GetSyncConstraintsExternalShape(
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double errorReduction)
        {
            var syncConstraints = new List<JacobianConstraint>();

            if (ExternalSyncShape != null)
            {
                var rotationAxisExt = ExternalSyncShape.RotationMatrix * RotationAxis;
                var hingeAxisExt = hingeAxis;

                syncConstraints.Add(JacobianCommon.GetDOF(
                    -1.0 * rotationAxisExt,
                    rotationAxis,
                    ExternalSyncShape,
                    ShapeB,
                    0.0,
                    0.0,
                    SpringCoefficientHingeAxis,
                    0.0,
                    ConstraintType.Joint));

                var ax = hingeAxis.Cross(rotationAxis).Normalize();
                var ax1 = hingeAxisExt.Cross(rotationAxisExt).Normalize();
                double error = (ax - ax1).Length() * errorReduction;

                syncConstraints.Add(JacobianCommon.GetDOF(
                    -1.0 * hingeAxis,
                    hingeAxis,
                    ExternalSyncShape,
                    ShapeB,
                    0.0,
                    error,
                    SpringCoefficientHingeAxis,
                    0.0,
                    ConstraintType.Joint));
            }

            return syncConstraints;
        }

        #endregion
    }
}

