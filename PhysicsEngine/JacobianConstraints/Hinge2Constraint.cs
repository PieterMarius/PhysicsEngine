using System;
using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
	public sealed class Hinge2Constraint: IConstraint, IConstraintBuilder
	{
		#region Private Fields

		const JointType jointType = JointType.Hinge2;

        IShape ShapeA;
        IShape ShapeB;
        int KeyIndex;
		readonly double SpringCoefficient;
		readonly double SpringCoefficientHingeAxis; 
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 HingeAxis;
		readonly Vector3 RotationAxis;
		readonly Vector3 StartErrorAxis1;
		readonly Vector3 StartErrorAxis2;
		readonly Quaternion RelativeOrientation1;
		readonly Quaternion RelativeOrientation2;

		double RestoreCoefficient;

		double? AngularLimitMin1;
		double? AngularLimitMax1;
		double? AngularLimitMin2;
		double? AngularLimitMax2;

		double? SpeedHingeAxisLimit;
		double? ForceHingeAxisLimit;
		double? SpeedRotationAxisLimit;
		double? ForceRotationAxisLimit;
		Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public Hinge2Constraint(
            IShape shapeA,
            IShape shapeB,
            Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double restoreCoefficient,
			double springCoefficientHingeAxis,
			double springCoefficient)
		{
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = GetHashCode();
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficientHingeAxis = springCoefficientHingeAxis;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;
			HingeAxis = hingeAxis.Normalize ();
			RotationAxis = rotationAxis.Normalize ();

			Vector3 relativePos = startAnchorPosition - ShapeA.StartPosition;
			relativePos = ShapeA.RotationMatrix * relativePos;

			AnchorPoint = relativePos + ShapeA.Position;

			StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeA.Position);

			StartErrorAxis2 = shapeB.RotationMatrix.Transpose() *
									 (AnchorPoint - shapeB.Position);

			Vector3 rHingeAxis = ShapeA.RotationMatrix * HingeAxis;
			Vector3 rRotationAxis = shapeB.RotationMatrix * RotationAxis;

			RelativeOrientation1 = CalculateRelativeOrientation(
				rHingeAxis,
				rRotationAxis,
                ShapeA.RotationStatus);

			RelativeOrientation2 = CalculateRelativeOrientation(
				rRotationAxis,
				rHingeAxis,
                shapeB.RotationStatus);
		}

		#endregion


		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the Universal joint.
		/// </summary>
		/// <returns>The Universal joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
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

			#region Base Constraint

			ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double constraintLimit = RestoreCoefficient * Vector3.Dot (t1,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
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

			//DOF 2

			constraintLimit = RestoreCoefficient * Vector3.Dot (tempPerpendicular,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
                tempPerpendicular,
				-1.0 * tempPerpendicular,
				Vector3.Cross (r1, tempPerpendicular),
				-1.0 * Vector3.Cross (r2, tempPerpendicular),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 3

			ConstraintType hingeAxisConstraintType = ConstraintType.Joint;
			if (SpringCoefficientHingeAxis > 0)
				hingeAxisConstraintType = ConstraintType.SoftJoint;

			constraintLimit = RestoreCoefficient * Vector3.Dot (hingeAxis,linearError);

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

			double angularLimit = RestoreCoefficient * (-k);

			hinge2Constraints.Add (
				JacobianCommon.GetDOF (
                    	t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
					SpringCoefficient,
					0.0,
					constraintType));

			#endregion

			#region Limit Constraints 

			hinge2Constraints.AddRange(GetAngularLimit(
				simulationObjectA,
				simulationObjectB,
				hingeAxis,
				rotationAxis));

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

		public JointType GetJointType()
		{
			return jointType;
		}

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

		public Vector3 GetAnchorPosition()
		{
			return (ShapeA.RotationMatrix *
                   (StartAnchorPoint - ShapeA.StartPosition)) +
                   ShapeA.Position;
        }

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

		public void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedHingeAxisLimit = speedValue;
			ForceHingeAxisLimit = forceLimit;
		}

		public void SetAxis2Motor(double speedValue, double forceLimit)
		{
			SpeedRotationAxisLimit = speedValue;
			ForceRotationAxisLimit = forceLimit;
		}

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin1 = angularLimitMin;
			AngularLimitMax1 = angularLimitMax;
		}

		public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin2 = angularLimitMin;
			AngularLimitMax2 = angularLimitMax;
		}

		public void AddTorque(double torqueAxis1, double torqueAxis2)
		{
			Vector3 hingeAxis = ShapeA.RotationMatrix * HingeAxis;
			Vector3 rotationAxis = ShapeB.RotationMatrix * RotationAxis;

			Vector3 torque = hingeAxis * torqueAxis1 + rotationAxis * torqueAxis2;

			ShapeA.SetTorque(ShapeA.TorqueValue + torque);
			ShapeB.SetTorque(ShapeB.TorqueValue - torque);
		}

		#region NotImplementedMethods

		void IConstraint.SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

		#region Private Methods

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
			Vector3 rotationAxis)
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
						RestoreCoefficient,
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
						RestoreCoefficient,
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

		List<JacobianConstraint> GetMotorConstraint(
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

		#endregion
	}
}

