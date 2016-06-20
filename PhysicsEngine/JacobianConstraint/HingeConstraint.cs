using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class HingeConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		private const JointType jointType = JointType.Hinge;

		private readonly int IndexA;
		private readonly int IndexB;
		private readonly double C;
		private readonly double K;
		private readonly Vector3 StartAnchorPoint;
		private readonly Vector3 HingeAxis;

		private readonly Vector3 StartErrorAxis1;
		private readonly Vector3 StartErrorAxis2;
		private readonly Quaternion RelativeOrientation;

		private double? AngularLimitMin = null;
		private double? AngularLimitMax = null;
		private double? SpeedValue = null;
		private double? ForceLimit = null;

		private Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public HingeConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			double K,
			double C)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectB.Position);

			this.RelativeOrientation = objectB.RotationStatus.Inverse () *
										objectA.RotationStatus;

			this.HingeAxis = hingeAxis.Normalize ();
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

			this.AnchorPoint = (simulationObjectA.RotationMatrix *
								(this.StartAnchorPoint -
								simulationObjectA.StartPosition)) +
								simulationObjectA.Position;

			#region Init Linear

			Vector3 axisRotated = simulationObjectA.RotationMatrix * this.HingeAxis;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (axisRotated).Normalize ();
			Vector3 t2 = Vector3.Cross (axisRotated, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				this.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				this.StartErrorAxis2;

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
				this.RelativeOrientation);

			#endregion

			#region Jacobian Constraint

			#region Base Constraint

			//DOF 1

			double constraintLimit = this.K * linearError.x;

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
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * linearError.y;

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
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * linearError.z;

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
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			double angularLimit = this.K *
				t1.Dot (angularError);

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
					C,
					0.0,
					ConstraintType.Joint));

			//DOF 5

			angularLimit = this.K *
				t2.Dot (angularError);

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
					C,
					0.0,
					ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			if (this.AngularLimitMin.HasValue && 
				this.AngularLimitMax.HasValue)
			{
				double angle = JacobianCommon.GetAngle (
					simulationObjectA,
					simulationObjectB,
					this.RelativeOrientation,
					this.HingeAxis);

				hingeConstraints.Add(JacobianCommon.GetAngularLimit (
					IndexA, 
					IndexB, 
					angle,
					this.K,
					C,
					simulationObjectA, 
					simulationObjectB, 
					axisRotated,
					this.AngularLimitMin.Value,
					this.AngularLimitMax.Value));
			}

			#endregion

			#region Motor Contraint

			if(this.SpeedValue.HasValue &&
				this.ForceLimit.HasValue)
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
						this.SpeedValue.Value,
						C,
						this.ForceLimit.Value,
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

		public JointType GetJointType()
		{
			return jointType;
		}

		public Vector3 GetAnchorPosition()
		{
			return this.AnchorPoint;
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

