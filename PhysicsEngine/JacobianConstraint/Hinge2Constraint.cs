using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public sealed class Hinge2Constraint: IConstraint, IConstraintBuilder
	{
		#region Private Fields

		const JointType jointType = JointType.Hinge2;

		int IndexA;
		int IndexB;
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
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double restoreCoefficient,
			double springCoefficientHingeAxis,
			double springCoefficient)
		{
			IndexA = indexA;
			IndexB = indexB;
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficientHingeAxis = springCoefficientHingeAxis;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;
			HingeAxis = hingeAxis.Normalize ();
			RotationAxis = rotationAxis.Normalize ();

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			AnchorPoint = relativePos + objectA.Position;

			StartErrorAxis1 = objectA.RotationMatrix.Transpose() *
									 (AnchorPoint - objectA.Position);

			StartErrorAxis2 = objectB.RotationMatrix.Transpose() *
									 (AnchorPoint - objectB.Position);

			Vector3 rHingeAxis = objectA.RotationMatrix * HingeAxis;
			Vector3 rRotationAxis = objectB.RotationMatrix * RotationAxis;

			RelativeOrientation1 = calculateRelativeOrientation(
				rHingeAxis,
				rRotationAxis,
				objectA.RotationStatus);

			RelativeOrientation2 = calculateRelativeOrientation(
				rRotationAxis,
				rHingeAxis,
				objectB.RotationStatus);
		}

		#endregion


		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the Universal joint.
		/// </summary>
		/// <returns>The Universal joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			var hinge2Constraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [IndexA];
			SimulationObject simulationObjectB = simulationObjs [IndexB];

			AnchorPoint = (simulationObjectA.RotationMatrix *
						  (StartAnchorPoint - simulationObjectA.StartPosition)) +
						  simulationObjectA.Position;

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

			//DOF 1

			double constraintLimit = RestoreCoefficient * Vector3.Dot (t1,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
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
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = RestoreCoefficient * Vector3.Dot (tempPerpendicular,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
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
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = RestoreCoefficient * Vector3.Dot (hingeAxis,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
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
				ConstraintType.Joint));
			
			//DOF 4

			double angularLimit = RestoreCoefficient * (-k);

			hinge2Constraints.Add (
				JacobianCommon.GetDOF (
					IndexA, 
					IndexB, 
					new Vector3(), 
					new Vector3(), 
					t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
					SpringCoefficient,
					0.0,
					ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			hinge2Constraints.AddRange(getAngularLimit(
				simulationObjectA,
				simulationObjectB,
				hingeAxis,
				rotationAxis));

			#endregion

			#region Motor Constraint

			hinge2Constraints.AddRange(getMotorConstraint(
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
			return IndexA;
		}

		public int GetObjectIndexB()
		{
			return IndexB;
		}

		public void SetObjectIndexA(int index)
		{
			IndexA = index;
		}

		public void SetObjectIndexB(int index)
		{
			IndexB = index;
		}

		public Vector3 GetAnchorPosition()
		{
			return AnchorPoint;
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

		public void AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			Vector3 hingeAxis = objects[IndexA].RotationMatrix * HingeAxis;
			Vector3 rotationAxis = objects[IndexB].RotationMatrix * RotationAxis;

			Vector3 torque = hingeAxis * torqueAxis1 + rotationAxis * torqueAxis2;

			objects[IndexA].SetTorque(objects[IndexA].TorqueValue + torque);
			objects[IndexB].SetTorque(objects[IndexB].TorqueValue - torque);
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

		double getAngle2(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			return -getAngle1(axis2, axis1, startAxis, rotationStatus, startRelativeRotation);
		}

		double getAngle1(
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

		Quaternion calculateRelativeOrientation(
			Vector3 axis1,
			Vector3 axis2,
			Quaternion bodyRotationStatus)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

			return Quaternion.Multiply1(bodyRotationStatus, rotationQ);
		}

		List<JacobianContact> getAngularLimit(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Vector3 hingeAxis,
			Vector3 rotationAxis)
		{
			var angularConstraint = new List<JacobianContact>();

			if (AngularLimitMin1.HasValue &&
				AngularLimitMax1.HasValue)
			{
				double angle1 = getAngle1(
					hingeAxis,
					rotationAxis,
					HingeAxis,
					simulationObjectA.RotationStatus,
					RelativeOrientation1);

				JacobianContact? jContact = 
					JacobianCommon.GetAngularLimit (
						IndexA,
						IndexB,
						angle1,
						RestoreCoefficient,
						SpringCoefficient,
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

				double angle2 = getAngle2(
					hingeAxis,
					rotationAxis,
					RotationAxis,
					simulationObjectB.RotationStatus,
					RelativeOrientation2);

				JacobianContact? jContact = 
					JacobianCommon.GetAngularLimit (
						IndexA,
						IndexB,
						angle2,
						RestoreCoefficient,
						SpringCoefficient,
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

		List<JacobianContact> getMotorConstraint(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Vector3 hingeAxis,
			Vector3 rotationAxis)
		{
			var motorConstraint = new List<JacobianContact>();

			if (SpeedHingeAxisLimit.HasValue &&
				ForceHingeAxisLimit.HasValue)
			{
				motorConstraint.Add(
					JacobianCommon.GetDOF(
						IndexA,
						IndexB,
						new Vector3(),
						new Vector3(),
						-1.0 * hingeAxis,
						1.0 * hingeAxis,
						simulationObjectA,
						simulationObjectB,
						SpeedHingeAxisLimit.Value,
						0.0,
						SpringCoefficient,
						ForceHingeAxisLimit.Value,
						ConstraintType.JointMotor));
			}

			if (SpeedRotationAxisLimit.HasValue &&
				ForceRotationAxisLimit.HasValue)
			{
				motorConstraint.Add(
					JacobianCommon.GetDOF(
						IndexA,
						IndexB,
						new Vector3(),
						new Vector3(),
						-1.0 * rotationAxis,
						1.0 * rotationAxis,
						simulationObjectA,
						simulationObjectB,
						SpeedRotationAxisLimit.Value,
						0.0,
						SpringCoefficient,
						ForceRotationAxisLimit.Value,
						ConstraintType.JointMotor));
			}

			return motorConstraint;
		}

		#endregion
	}
}

