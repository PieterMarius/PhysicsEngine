using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class FixedJointConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		private const JointType jointType = JointType.Fixed;

		private readonly int IndexA;
		private readonly int IndexB;
		private readonly double C;
		private readonly double K;
		private readonly Vector3 StartAnchorPoint;

		private Vector3 AnchorPoint;
		private Vector3 StartErrorAxis1;
		private Vector3 StartErrorAxis2;
		private Quaternion RelativeOrientation;

		#endregion

		#region Constructor

		public FixedJointConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			double K,
			double C)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.C = C;
			this.K = K;

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			this.StartAnchorPoint = (objectB.Position - objectA.Position) * 0.5;

			Vector3 relativePos = this.StartAnchorPoint - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectB.Position);

			this.RelativeOrientation = objectB.RotationStatus.Inverse () *
									   objectA.RotationStatus;
		}

		#endregion

		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the fixed joint.
		/// </summary>
		/// <returns>The fixed joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			List<JacobianContact> fixedConstraints = new List<JacobianContact> ();

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

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix ();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix ();

			#endregion

			#region Init Angular

			Vector3 angularError = JacobianCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				this.RelativeOrientation);

			#endregion

			#region Jacobian Constraint

			double constraintLimit = this.K * linearError.x;

			//DOF 1

			fixedConstraints.Add (JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3 (skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * linearError.y;

			fixedConstraints.Add (JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3 (skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * linearError.z;

			fixedConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3 (skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = this.K * 2.0 * angularError.x;

			fixedConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 5

			constraintLimit = this.K * 2.0 * angularError.y;

			fixedConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 6

			constraintLimit = this.K * 2.0 * angularError.z;

			fixedConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (0.0, 0.0, 1.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			#endregion

			return fixedConstraints;
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

		public Vector3 GetStartAnchorPosition()
		{
			return this.StartAnchorPoint;
		}

		public Vector3 GetAnchorPosition()
		{
			return this.AnchorPoint;
		}

		#region NotSupportedMethods

		void IConstraint.SetAxis1Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

		void IConstraint.SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

		void IConstraint.SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			throw new NotSupportedException();
		}

		void IConstraint.SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			throw new NotSupportedException();
		}

		public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotImplementedException();
		}

		public void AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			throw new NotImplementedException();
		}

		#endregion

		#endregion

		#endregion
	}
}

