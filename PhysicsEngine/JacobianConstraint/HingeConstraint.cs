using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class HingeConstraint: IConstraint, IConstraintBuilder
	{
		#region Private Fields

		const JointType jointType = JointType.Hinge;

		int IndexA;
		int IndexB;
		readonly double SpringCoefficient;
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 HingeAxis;
		readonly Vector3 StartErrorAxis1;
		readonly Vector3 StartErrorAxis2;
		readonly Quaternion RelativeOrientation;

		double? AngularLimitMin = null;
		double? AngularLimitMax = null;
		double? SpeedValue = null;
		double? ForceLimit = null;

		double RestoreCoefficient;
		Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public HingeConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			double restoreCoefficient,
			double springCoefficient)
		{
			IndexA = indexA;
			IndexB = indexB;
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			AnchorPoint = relativePos + objectA.Position;

			StartErrorAxis1 = objectA.RotationMatrix.Transpose() *
									 (AnchorPoint - objectA.Position);

			StartErrorAxis2 = objectB.RotationMatrix.Transpose() *
									 (AnchorPoint - objectB.Position);

			RelativeOrientation = objectB.RotationStatus.Inverse () *
										objectA.RotationStatus;

			HingeAxis = hingeAxis.Normalize ();
		}

		#endregion

		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the hinge joint.
		/// </summary>
		/// <returns>The hinge joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			List<JacobianContact> hingeConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [IndexA];
			SimulationObject simulationObjectB = simulationObjs [IndexB];

			AnchorPoint = (simulationObjectA.RotationMatrix *
						  (StartAnchorPoint - simulationObjectA.StartPosition)) +
						  simulationObjectA.Position;

			#region Init Linear

			Vector3 axisRotated = simulationObjectA.RotationMatrix * HingeAxis;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (axisRotated).Normalize ();
			Vector3 t2 = Vector3.Cross (axisRotated, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Matrix3x3 skewP1 = Matrix3x3.GetSkewSymmetricMatrix (r1);
			Matrix3x3 skewP2 = Matrix3x3.GetSkewSymmetricMatrix (r2);

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 angularError = JacobianCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				RelativeOrientation);

			#endregion

			#region Jacobian Constraint

			#region Base Constraint

			//DOF 1

			double constraintLimit = RestoreCoefficient * linearError.x;

			hingeConstraints.Add (JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (-skewP1.r1c1, -skewP1.r1c2, -skewP1.r1c3),
				new Vector3 (skewP2.r1c1,skewP2.r1c2,skewP2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = RestoreCoefficient * linearError.y;

			hingeConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (-skewP1.r2c1, -skewP1.r2c2, -skewP1.r2c3),
				new Vector3 (skewP2.r2c1,skewP2.r2c2,skewP2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = RestoreCoefficient * linearError.z;

			hingeConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (-skewP1.r3c1, -skewP1.r3c2, -skewP1.r3c3),
				new Vector3 (skewP2.r3c1,skewP2.r3c2,skewP2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			double angularLimit = RestoreCoefficient *
				t1.Dot(angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
					IndexA, 
					IndexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t1, 
					1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					SpringCoefficient,
					0.0,
					ConstraintType.Joint));

			//DOF 5

			angularLimit = RestoreCoefficient *
				t2.Dot(angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
					IndexA, 
					IndexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t2, 
					1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit,
					SpringCoefficient,
					0.0,
					ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			if (AngularLimitMin.HasValue && 
				AngularLimitMax.HasValue)
			{
				double angle = JacobianCommon.GetAngle (
					simulationObjectA,
					simulationObjectB,
					RelativeOrientation,
					HingeAxis);

				JacobianContact? jContact = 
					JacobianCommon.GetAngularLimit (
						IndexA, 
						IndexB, 
						angle,
						RestoreCoefficient,
						SpringCoefficient,
						simulationObjectA, 
						simulationObjectB, 
						axisRotated,
						AngularLimitMin.Value,
						AngularLimitMax.Value);

				if (jContact != null)
					hingeConstraints.Add (jContact.Value);
			}

			#endregion

			#region Motor Contraint

			if(SpeedValue.HasValue &&
				ForceLimit.HasValue)
			{
				hingeConstraints.Add (
					JacobianCommon.GetDOF (
						IndexA, 
						IndexB, 
						new Vector3(), 
						new Vector3(), 
						-1.0 * axisRotated, 
						1.0 * axisRotated, 
						simulationObjectA, 
						simulationObjectB, 
						SpeedValue.Value,
						SpringCoefficient,
						ForceLimit.Value,
						ConstraintType.JointMotor));
			}

			#endregion

			#endregion

			return hingeConstraints;
		}

		#endregion

		#region IConstraint

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

		public JointType GetJointType()
		{
			return jointType;
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
			SpeedValue = speedValue;
			ForceLimit = forceLimit;
		}

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin = angularLimitMin;
			AngularLimitMax = angularLimitMax;
		}

		public void AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			Vector3 hingeAxis = objects[IndexA].RotationMatrix * this.HingeAxis;

			Vector3 torque = hingeAxis * torqueAxis1;

			objects[IndexA].SetTorque(objects[IndexA].TorqueValue + torque);
			objects[IndexB].SetTorque(objects[IndexB].TorqueValue - torque);
		}

		#region NotImplementedMethods

		void IConstraint.SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

		void IConstraint.SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			throw new NotSupportedException();
		}

		void IConstraint.SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotImplementedException();
		}

		void IConstraint.AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			throw new NotImplementedException();
		}

		#endregion

		#endregion

		#endregion

	}
}

