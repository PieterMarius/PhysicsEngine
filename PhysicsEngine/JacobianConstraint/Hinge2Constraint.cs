using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public sealed class Hinge2Constraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		private const JointType jointType = JointType.Hinge2;

		private readonly int IndexA;
		private readonly int IndexB;
		private readonly double C;
		private readonly double K;
		private readonly double KHingeAxis;
		private readonly Vector3 StartAnchorPoint;
		private readonly Vector3 HingeAxis;
		private readonly Vector3 RotationAxis;
		private readonly Vector3 StartErrorAxis1;
		private readonly Vector3 StartErrorAxis2;
		private readonly Quaternion RelativeOrientation1;
		private readonly Quaternion RelativeOrientation2;

		private double? AngularLimitMin1 = null;
		private double? AngularLimitMax1 = null;
		private double? AngularLimitMin2 = null;
		private double? AngularLimitMax2 = null;

		private double? SpeedHingeAxisLimit = null;
		private double? ForceHingeAxisLimit = null;
		private double? SpeedRotationAxisLimit = null;
		private double? ForceRotationAxisLimit = null;
		private Vector3 AnchorPoint;


		#endregion

		#region Constructor

		public Hinge2Constraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double K,
			double KHingeAxis,
			double C)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.K = K;
			this.KHingeAxis = KHingeAxis;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;
			this.HingeAxis = hingeAxis.Normalize ();
			this.RotationAxis = rotationAxis.Normalize ();

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose () *
				(this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose () *
				(this.AnchorPoint - objectB.Position);

			Vector3 rHingeAxis = objectA.RotationMatrix * this.HingeAxis;
			Vector3 rRotationAxis = objectB.RotationMatrix * this.RotationAxis;

			this.RelativeOrientation1 = calculateRelativeOrientation (
				rHingeAxis,
				rRotationAxis,
				objectA.RotationStatus);

			this.RelativeOrientation2 = calculateRelativeOrientation (
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
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			List<JacobianContact> hinge2Constraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [IndexA];
			SimulationObject simulationObjectB = simulationObjs [IndexB];

			this.AnchorPoint = (simulationObjectA.RotationMatrix *
				(this.StartAnchorPoint -
					simulationObjectA.StartPosition)) +
				simulationObjectA.Position;

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
				this.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				this.StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 hingeAxis = simulationObjectA.RotationMatrix * this.HingeAxis;
			Vector3 rotationAxis = simulationObjectB.RotationMatrix * this.RotationAxis;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3 t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

			#endregion

			#region Jacobian Constraint

			#region Base Constraint

			//DOF 1

			double constraintLimit = this.K * Vector3.Dot (t1,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * Vector3.Dot (tempPerpendicular,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				tempPerpendicular,
				-1.0 * tempPerpendicular,
				Vector3.Cross (r1, tempPerpendicular),
				-1.0 * Vector3.Cross (r2, tempPerpendicular),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * Vector3.Dot (hingeAxis,linearError);

			hinge2Constraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				hingeAxis,
				-1.0 * hingeAxis,
				Vector3.Cross (r1, hingeAxis),
				-1.0 * Vector3.Cross (r2, hingeAxis),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				KHingeAxis,
				0.0,
				ConstraintType.Joint));
			
			//DOF 4

			double angularLimit = this.K * (-k);

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
					angularLimit,
					C,
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

		public Vector3 GetAnchorPosition()
		{
			return this.AnchorPoint;
		}

		public void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedHingeAxisLimit = speedValue;
			ForceHingeAxisLimit = forceLimit;
		}

		public void SetAxis2Motor(double speedValue, double forceLimit)
		{
			this.SpeedRotationAxisLimit = speedValue;
			this.ForceRotationAxisLimit = forceLimit;
		}

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			this.AngularLimitMin1 = angularLimitMin;
			this.AngularLimitMax1 = angularLimitMax;
		}

		public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			this.AngularLimitMin2 = angularLimitMin;
			this.AngularLimitMax2 = angularLimitMax;
		}

		public void AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			//Vector3 hingeAxis = objects[IndexA].RotationMatrix * this.HingeAxis;
			Vector3 rotationAxis = objects[IndexB].RotationMatrix * this.RotationAxis;

			Vector3 torque = rotationAxis * torqueAxis2;

			objects[IndexB].SetTorque(torque);
		}

		#region NotImplementedMethods

		void IConstraint.SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotImplementedException();
		}

		#endregion

		#endregion

		#endregion

		#region Private Methods

		private double getAngle2(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			return -getAngle1(axis2, axis1, startAxis, rotationStatus, startRelativeRotation);
		}

		private double getAngle1(
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

			Vector3 quaternionVectorPart = new Vector3(
				mult2.b,
				mult2.c,
				mult2.d);

			return JacobianCommon.GetRotationAngle(quaternionVectorPart, mult2.a, startAxis);
		}

		private Quaternion calculateRelativeOrientation(
			Vector3 axis1,
			Vector3 axis2,
			Quaternion bodyRotationStatus)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix (axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion (rotationMatrix);

			return Quaternion.Multiply1 (bodyRotationStatus, rotationQ);
		}

		private List<JacobianContact> getAngularLimit(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Vector3 hingeAxis,
			Vector3 rotationAxis)
		{
			var angularConstraint = new List<JacobianContact>();

			if (this.AngularLimitMin1.HasValue &&
				this.AngularLimitMax1.HasValue)
			{
				double angle1 = getAngle1(
					hingeAxis,
					rotationAxis,
					this.HingeAxis,
					simulationObjectA.RotationStatus,
					this.RelativeOrientation1);

				angularConstraint.Add(JacobianCommon.GetAngularLimit(
					IndexA,
					IndexB,
					angle1,
					this.K,
					C,
					simulationObjectA,
					simulationObjectB,
					hingeAxis,
					this.AngularLimitMin1.Value,
					this.AngularLimitMax1.Value));
			}

			if (this.AngularLimitMin2.HasValue &&
				this.AngularLimitMax2.HasValue)
			{

				double angle2 = getAngle2(
					hingeAxis,
					rotationAxis,
					this.RotationAxis,
					simulationObjectB.RotationStatus,
					this.RelativeOrientation2);

				angularConstraint.Add(JacobianCommon.GetAngularLimit(
					IndexA,
					IndexB,
					angle2,
					this.K,
					C,
					simulationObjectA,
					simulationObjectB,
					rotationAxis,
					this.AngularLimitMin2.Value,
					this.AngularLimitMax2.Value));

			}

			return angularConstraint;
		}

		private List<JacobianContact> getMotorConstraint(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Vector3 hingeAxis,
			Vector3 rotationAxis)
		{
			var motorConstraint = new List<JacobianContact>();

			if (this.SpeedHingeAxisLimit.HasValue &&
				this.ForceHingeAxisLimit.HasValue)
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
						this.SpeedHingeAxisLimit.Value,
						C,
						this.ForceHingeAxisLimit.Value,
						ConstraintType.JointMotor));
			}

			if (this.SpeedRotationAxisLimit.HasValue &&
				this.ForceRotationAxisLimit.HasValue)
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
						this.SpeedRotationAxisLimit.Value,
						C,
						this.ForceRotationAxisLimit.Value,
						ConstraintType.JointMotor));
			}

			return motorConstraint;
		}

		#endregion
	}
}

