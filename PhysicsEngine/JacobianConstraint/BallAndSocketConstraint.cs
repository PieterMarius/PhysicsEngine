using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class BallAndSocketConstraint : IConstraint, IConstraintBuilder
	{
		#region Public Fields

		private const JointType jointType = JointType.BallAndSocket;

		private readonly int IndexA;
		private readonly int IndexB;
		private readonly double C;
		private readonly double K;
		private readonly Vector3 StartAnchorPoint;

		private readonly Vector3 StartErrorAxis1;
		private readonly Vector3 StartErrorAxis2;

		private Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public BallAndSocketConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
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

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose() *
									(this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose() *
									(this.AnchorPoint - objectB.Position);
		}

		#endregion

		#region Public Methods

		#region IConstraintBuider

		/// <summary>
		/// Builds the ball socket joint.
		/// </summary>
		/// <returns>The ball socket joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			List<JacobianContact> ballSocketConstraints = new List<JacobianContact>();

			SimulationObject simulationObjectA = simulationObjs[IndexA];
			SimulationObject simulationObjectB = simulationObjs[IndexB];

			this.AnchorPoint = (simulationObjectA.RotationMatrix *
								(this.StartAnchorPoint -
								simulationObjectA.StartPosition)) +
								simulationObjectA.Position;

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
				this.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				this.StartErrorAxis2;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix();

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Jacobian Constraint

			double constraintLimit = this.K * linearError.x;

			//DOF 1

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3(1.0, 0.0, 0.0),
				new Vector3(-1.0, 0.0, 0.0),
				new Vector3(-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3(skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * linearError.y;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3(0.0, 1.0, 0.0),
				new Vector3(0.0, -1.0, 0.0),
				new Vector3(-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3(skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * linearError.z;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3(0.0, 0.0, 1.0),
				new Vector3(0.0, 0.0, -1.0),
				new Vector3(-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3(skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			#endregion

			return ballSocketConstraints;
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

