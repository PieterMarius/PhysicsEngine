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
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine
{
    internal sealed class PistonConstraint: Constraint
	{
		#region Public Fields

		const JointType jointType = JointType.Piston;

        readonly Vector3d StartAnchorPoint;
		readonly Vector3d PistonAxis;

		double? AngularLimitMin;
		double? AngularLimitMax;

		double? LinearLimitMin;
		double? LinearLimitMax;

		double? LinearSpeedValue;
		double? LinearForceLimit;
		double? AngularSpeedValue;
		double? AngularForceLimit;

		Vector3d AnchorPoint;
		Vector3d StartErrorAxis1;
		Vector3d StartErrorAxis2;
		Quaternion RelativeOrientation;

        #endregion

        #region Constructor

        public PistonConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3d startAnchorPosition,
            Vector3d pistonAxis,
            double errorReductionParam,
            double springCoefficient)
            : base(shapeA, shapeB, errorReductionParam, springCoefficient)
        {
            StartAnchorPoint = startAnchorPosition;

            PistonAxis = -1.0 * pistonAxis.Normalize();

            Vector3d relativePos = ShapeA.RotationMatrix *
                (startAnchorPosition - ShapeA.InitCenterOfMass);

            AnchorPoint = relativePos + ShapeA.Position;

            StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeA.Position);

            StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeB.Position);

            RelativeOrientation = ShapeB.RotationStatus.Inverse() *
                                         ShapeA.RotationStatus;
        }

		#endregion

		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the piston joint.
		/// </summary>
		/// <returns>The piston joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public override List<JacobianConstraint> BuildJacobian()
		{
			var pistonConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            			
			#region Init Linear

			Vector3d sliderAxis = GetSliderAxis();

			Vector3d t1 = GeometryUtils.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3d t2 = Vector3d.Cross (sliderAxis, t1).Normalize ();

			Vector3d r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3d r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Vector3d p1 = simulationObjectA.Position + r1;
			Vector3d p2 = simulationObjectB.Position + r2;

			Vector3d linearError = p2 - p1;

			#endregion

			Vector3d angularError = sliderAxis.Cross (
				                       (simulationObjectB.RotationMatrix * PistonAxis));

            #region Jacobian Constraint

            double errorReduction = ErrorReductionParam;
            double springCoefficient = SpringCoefficient;

            #region Base Constraints

            ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double angularLimit = errorReduction *
				t1.Dot (angularError);

			pistonConstraints.Add (
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

			//DOF 2

			angularLimit = errorReduction *
				t2.Dot (angularError);

			pistonConstraints.Add (
				JacobianCommon.GetDOF (
                    t2, 
					-1.0 * t2, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
                    springCoefficient,
					0.0,
					constraintType));

			//DOF 3

			double constraintLimit = errorReduction * Vector3d.Dot (t1,linearError);

			pistonConstraints.Add (JacobianCommon.GetDOF (
                t1,
				-1.0 * t1,
				Vector3d.Cross (r1, t1),
				-1.0 * Vector3d.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 4

			constraintLimit = errorReduction * Vector3d.Dot (t2,linearError);

			pistonConstraints.Add (JacobianCommon.GetDOF (
                t2,
				-1.0 * t2,
				Vector3d.Cross (r1, t2),
				-1.0 * Vector3d.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			#endregion

			#region Limit Constraints 

			pistonConstraints.AddRange(GetLinearLimit(
				simulationObjectA,
				simulationObjectB,
				sliderAxis,
				r1,
				r2,
                errorReduction));

			pistonConstraints.AddRange(GetAnguarLimit(
				simulationObjectA,
				simulationObjectB,
				sliderAxis,
                errorReduction));

			#endregion

			#region Motor Constraint

			pistonConstraints.AddRange(GetMotorConstraint(
				simulationObjectA,
				simulationObjectB,
				sliderAxis));

			#endregion

			#endregion

			return pistonConstraints;
		}

        #endregion

        #region IConstraint

        public override JointType GetJointType()
		{
			return jointType;
		}

		public override Vector3d GetAnchorPosition()
		{
			return (ShapeA.RotationMatrix *
                    (StartAnchorPoint -
                    ShapeA.InitCenterOfMass)) +
                    ShapeA.Position;
        }

		public override void SetAxis1AngularLimit(double? angularLimitMin, double? angularLimitMax)
		{
			AngularLimitMin = angularLimitMin;
			AngularLimitMax = angularLimitMax;
		}

		public override void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			LinearLimitMin = linearLimitMin;
			LinearLimitMax = linearLimitMax;
		}

		public override void SetAxis1Motor(double speedValue, double forceLimit)
		{
			LinearSpeedValue = speedValue;
			LinearForceLimit = forceLimit;
		}

		/// <summary>
		/// Sets the rotation motor.
		/// </summary>
		/// <returns>The axis2 motor.</returns>
		/// <param name="speedValue">Speed value.</param>
		/// <param name="forceLimit">Force limit.</param>
		public override void SetAxis2Motor(double speedValue, double forceLimit)
		{
			AngularSpeedValue = speedValue;
			AngularForceLimit = forceLimit;
		}
        
		public override void AddTorque(double torqueAxis1, double torqueAxis2)
		{
			Vector3d pistonAxis = ShapeA.RotationMatrix * PistonAxis;

			Vector3d torque = PistonAxis * torqueAxis1;

            ShapeA.SetTorque(ShapeA.TorqueValue + torque);
            ShapeB.SetTorque(ShapeB.TorqueValue - torque);
		}

        public Vector3d GetSliderAxis()
        {
            return ShapeA.RotationMatrix * PistonAxis;
        }
                
        #region NotImplementedMethod

        public override void SetAxis2AngularLimit(double? angularLimitMin, double? angularLimitMax)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

		#region Private Methods

		List<JacobianConstraint> GetLinearLimit(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3d sliderAxis,
			Vector3d r1,
			Vector3d r2,
            double errorReduction)
		{
			var linearConstraints = new List<JacobianConstraint>();

			if (LinearLimitMin.HasValue &&
				LinearLimitMax.HasValue)
			{
				linearConstraints.Add(
					JacobianCommon.GetLinearLimit(
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						r1,
						r2,
                        errorReduction,
						0.0,
						LinearLimitMin.Value,
						LinearLimitMax.Value));
			}

			return linearConstraints;
		}

		List<JacobianConstraint> GetAnguarLimit(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3d sliderAxis,
            double errorReduction)
		{
			var angularConstraints = new List<JacobianConstraint>();

			if (AngularLimitMin.HasValue &&
				AngularLimitMax.HasValue)
			{
				double angle = JacobianCommon.GetAngle(
					simulationObjectA,
					simulationObjectB,
					RelativeOrientation,
					PistonAxis);

				JacobianConstraint? jContact = 
					JacobianCommon.GetAngularLimit (
						angle,
                        errorReduction,
						0.0,
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						AngularLimitMin.Value,
						AngularLimitMax.Value);

				if (jContact != null)
					angularConstraints.Add (jContact.Value);
			}

			return angularConstraints;
		}

		List<JacobianConstraint> GetMotorConstraint(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3d sliderAxis)
		{
			var motorConstraints = new List<JacobianConstraint>();

			if (LinearForceLimit.HasValue &&
				LinearSpeedValue.HasValue)
			{
				motorConstraints.Add(JacobianCommon.GetDOF(
					sliderAxis,
					-1.0 * sliderAxis,
					new Vector3d(),
					new Vector3d(),
					simulationObjectA,
					simulationObjectB,
					LinearSpeedValue.Value,
					0.0,
					0.0,
					LinearForceLimit.Value,
					ConstraintType.JointMotor));
			}

			if (AngularForceLimit.HasValue &&
			   AngularSpeedValue.HasValue)
			{
				motorConstraints.Add(JacobianCommon.GetDOF(
					new Vector3d(),
					new Vector3d(),
					sliderAxis,
					-1.0 * sliderAxis,
					simulationObjectA,
					simulationObjectB,
					AngularSpeedValue.Value,
					0.0,
					0.0,
					AngularForceLimit.Value,
					ConstraintType.JointMotor));
			}

			return motorConstraints;
		}



		#endregion
	}
}

