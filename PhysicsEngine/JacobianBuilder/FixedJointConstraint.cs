﻿using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public static class FixedJointConstraint
	{
		/// <summary>
		/// Sets the joint.
		/// </summary>
		/// <returns>The joint.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="K">K.</param>
		/// <param name="C">C.</param>
		public static Joint SetJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			double K,
			double C)
		{
			Vector3 startAnchorPosition = (objectB.Position - objectA.Position) * 0.5;

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
			                        (anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
			                        (anchorPosition - objectB.Position);

			Quaternion relativeOrientation = Quaternion.Inverse (objectA.RotationStatus) *
			                                 objectB.RotationStatus;

			Joint joint = new Joint (
				K,
				C,
				JointType.Fixed,
				startAnchorPosition,
				anchorPosition,
				distanceFromA,
				distanceFromB,
				relativeOrientation,
				new Quaternion (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 ());

			return joint;
		}

		/// <summary>
		/// Builds the fixed joint.
		/// </summary>
		/// <returns>The fixed joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public static List<JacobianContact> BuildJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> fixedConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix ();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix ();

			#endregion

			#region Init Angular

			Vector3 angularError = JacobianBuilderCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				simulationJoint);

			#endregion

			#region Jacobian Constraint

			double constraintLimit = simulationJoint.K * linearError.x;

			//DOF 1

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF(
				indexA,
				indexB,
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3 (skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = simulationJoint.K * linearError.y;

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF(
				indexA,
				indexB,
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3 (skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = simulationJoint.K * linearError.z;

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3 (skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = simulationJoint.K * 2.0 * angularError.x;

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 5

			constraintLimit = simulationJoint.K * 2.0 * angularError.y;

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 6

			constraintLimit = simulationJoint.K * 2.0 * angularError.z;

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			#endregion

			return fixedConstraints;
		}
	}
}
