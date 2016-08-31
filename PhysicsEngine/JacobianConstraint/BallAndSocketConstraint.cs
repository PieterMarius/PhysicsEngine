using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class BallAndSocketConstraint : IConstraint, IConstraintBuilder
	{
		#region Public Fields

		const JointType jointType = JointType.BallAndSocket;

		int IndexA;
		int IndexB;
		int KeyIndex;
		readonly double SpringCoefficient;
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 StartErrorAxis1;
		readonly Vector3 StartErrorAxis2;

		double RestoreCoefficient;
		Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public BallAndSocketConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			double restoreCoefficient,
			double springCoefficient)
		{
			IndexA = indexA;
			IndexB = indexB;
			KeyIndex = this.GetHashCode();
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
		}

		#endregion

		#region Public Methods

		#region IConstraintBuider

		/// <summary>
		/// Builds the ball socket joint.
		/// </summary>
		/// <returns>The ball socket joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			var ballSocketConstraints = new List<JacobianContact>();

			SimulationObject simulationObjectA = simulationObjs[IndexA];
			SimulationObject simulationObjectB = simulationObjs[IndexB];

			AnchorPoint = (simulationObjectA.RotationMatrix *
						  (StartAnchorPoint - simulationObjectA.StartPosition)) +
						  simulationObjectA.Position;

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix();

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			//TODO test
			Console.WriteLine("Linear error: " + linearError.x + " " + linearError.y + " " + linearError.z + " " + linearError.Length());

			#endregion

			#region Jacobian Constraint

			double constraintLimit = RestoreCoefficient * linearError.x;

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
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = RestoreCoefficient * linearError.y;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3(0.0, 1.0, 0.0),
				new Vector3(0.0, -1.0, 0.0),
				new Vector3(-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3(skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = RestoreCoefficient * linearError.z;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
				IndexA,
				IndexB,
				new Vector3(0.0, 0.0, 1.0),
				new Vector3(0.0, 0.0, -1.0),
				new Vector3(-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3(skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
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

		public void SetObjectIndexA(int index)
		{
			IndexA = index;
		}

		public void SetObjectIndexB(int index)
		{
			IndexB = index;
		}

		public int GetObjectIndexB()
		{
			return IndexB;
		}

		public int GetKeyIndex()
		{
			return KeyIndex;
		}

		public Vector3 GetAnchorPosition()
		{
			return AnchorPoint;
		}

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
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

		void IConstraint.SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotSupportedException();
		}

		void IConstraint.AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion
	}
}

