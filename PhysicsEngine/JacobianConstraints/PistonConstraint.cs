using System;
using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine
{
	public sealed class PistonConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		const JointType jointType = JointType.Piston;

        IShape ShapeA;
        IShape ShapeB;
        int KeyIndex;
		readonly double SpringCoefficient;
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 PistonAxis;

		double? AngularLimitMin;
		double? AngularLimitMax;

		double? LinearLimitMin;
		double? LinearLimitMax;

		double? LinearSpeedValue;
		double? LinearForceLimit;
		double? AngularSpeedValue;
		double? AngularForceLimit;

		double RestoreCoefficient;
		Vector3 AnchorPoint;
		Vector3 StartErrorAxis1;
		Vector3 StartErrorAxis2;
		Quaternion RelativeOrientation;

		#endregion

		#region Constructor

		public PistonConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3 startAnchorPosition,
			Vector3 pistonAxis,
			double restoreCoefficient,
			double springCoefficient)
		{
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = GetHashCode();
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;

			PistonAxis = -1.0 * pistonAxis.Normalize ();

			Vector3 relativePos = ShapeA.RotationMatrix *
				(startAnchorPosition - ShapeA.StartPosition);

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
		public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
		{
			var pistonConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            			
			#region Init Linear

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * PistonAxis;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3 t2 = Vector3.Cross (sliderAxis, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			Vector3 angularError = sliderAxis.Cross (
				                       (simulationObjectB.RotationMatrix * PistonAxis));

			#region Jacobian Constraint

			#region Base Constraints

			ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double angularLimit = RestoreCoefficient *
				t1.Dot (angularError);

			pistonConstraints.Add (
				JacobianCommon.GetDOF (
					1.0 * t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
					SpringCoefficient,
					0.0,
					constraintType));

			//DOF 2

			angularLimit = RestoreCoefficient *
				t2.Dot (angularError);

			pistonConstraints.Add (
				JacobianCommon.GetDOF (
                    	1.0 * t2, 
					-1.0 * t2, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
					SpringCoefficient,
					0.0,
					constraintType));

			//DOF 3

			double constraintLimit = RestoreCoefficient * Vector3.Dot (t1,linearError);

			pistonConstraints.Add (JacobianCommon.GetDOF (
                t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 4

			constraintLimit = RestoreCoefficient * Vector3.Dot (t2,linearError);

			pistonConstraints.Add (JacobianCommon.GetDOF (
                t2,
				-1.0 * t2,
				Vector3.Cross (r1, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			#endregion

			#region Limit Constraints 

			pistonConstraints.AddRange(getLinearLimit(
				simulationObjectA,
				simulationObjectB,
				sliderAxis,
				r1,
				r2));

			pistonConstraints.AddRange(getAnguarLimit(
				simulationObjectA,
				simulationObjectB,
				sliderAxis));

			#endregion

			#region Motor Constraint

			pistonConstraints.AddRange(getMotorConstraint(
				simulationObjectA,
				simulationObjectB,
				sliderAxis));

			#endregion

			#endregion

			return pistonConstraints;
		}

		#endregion

		#region IConstraint

		public int GetObjectIndexA()
		{
			return ShapeA.ID;
		}

		public int GetObjectIndexB()
		{
			return ShapeB.ID;
		}

		public int GetKeyIndex()
		{
			return KeyIndex;
		}

		public JointType GetJointType()
		{
			return jointType;
		}

		public Vector3 GetAnchorPosition()
		{
			return (ShapeA.RotationMatrix *
                    (StartAnchorPoint -
                    ShapeA.StartPosition)) +
                    ShapeA.Position;
        }

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin = angularLimitMin;
			AngularLimitMax = angularLimitMax;
		}

		public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			LinearLimitMin = linearLimitMin;
			LinearLimitMax = linearLimitMax;
		}

		public void SetAxis1Motor(double speedValue, double forceLimit)
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
		public void SetAxis2Motor(double speedValue, double forceLimit)
		{
			AngularSpeedValue = speedValue;
			AngularForceLimit = forceLimit;
		}

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

		public void AddTorque(double torqueAxis1, double torqueAxis2)
		{
			Vector3 pistonAxis = ShapeA.RotationMatrix * PistonAxis;

			Vector3 torque = PistonAxis * torqueAxis1;

            ShapeA.SetTorque(ShapeA.TorqueValue + torque);
            ShapeB.SetTorque(ShapeB.TorqueValue - torque);
		}

		#region NotImplementedMethod

		void IConstraint.SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

		#region Private Methods

		List<JacobianConstraint> getLinearLimit(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3 sliderAxis,
			Vector3 r1,
			Vector3 r2)
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
						RestoreCoefficient,
						0.0,
						LinearLimitMin.Value,
						LinearLimitMax.Value));
			}

			return linearConstraints;
		}

		List<JacobianConstraint> getAnguarLimit(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3 sliderAxis)
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
						RestoreCoefficient,
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

		List<JacobianConstraint> getMotorConstraint(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3 sliderAxis)
		{
			var motorConstraints = new List<JacobianConstraint>();

			if (LinearForceLimit.HasValue &&
				LinearSpeedValue.HasValue)
			{
				motorConstraints.Add(JacobianCommon.GetDOF(
					sliderAxis,
					-1.0 * sliderAxis,
					new Vector3(),
					new Vector3(),
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
					new Vector3(),
					new Vector3(),
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

