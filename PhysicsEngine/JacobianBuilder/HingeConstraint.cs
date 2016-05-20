using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public static class HingeConstraint
	{
		/// <summary>
		/// Sets the hinge joint.
		/// </summary>
		/// <returns>The hinge joint.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="hingeAxis">Hinge axis.</param>
		/// <param name="K">K.</param>
		/// <param name="C">C.</param>
		/// <param name="angularLimitMin">Angular limit minimum.</param>
		/// <param name="angularLimitMax">Angular limit max.</param>
		/// <param name="startAnchorPosition">Start anchor position (default: objectB.Position - objectA.Position * 0.5).</param>
		public static Joint SetJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			double K,
			double C,
			double angularLimitMin = 0.0,
			double angularLimitMax = 0.0)
		{

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
			                        (anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
			                        (anchorPosition - objectB.Position);

			Quaternion relativeOrientation = objectB.RotationStatus.Inverse () *
			                                 objectA.RotationStatus;

			hingeAxis = hingeAxis.Normalize ();

			Vector3 angularLimitMinVec = new Vector3 ();
			Vector3 angularLimitMaxVec = new Vector3 ();

			if (angularLimitMin != angularLimitMax) {
				angularLimitMinVec = hingeAxis * angularLimitMin;
				angularLimitMaxVec = hingeAxis * angularLimitMax;
			}

			Joint joint = new Joint (
				              K,
				              C,
				              JointType.Hinge,
				              startAnchorPosition,
				              anchorPosition,
				              distanceFromA,
				              distanceFromB,
				              relativeOrientation,
				              new Quaternion (),
				              hingeAxis,
				              new Vector3 (),
				              new Vector3 (),
				              new Vector3 (),
				              angularLimitMinVec,
				              angularLimitMaxVec);

			return joint;
		}

		/// <summary>
		/// Builds the hinge joint.
		/// </summary>
		/// <returns>The hinge joint.</returns>
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
			List<JacobianContact> hingeConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 axisRotated = simulationObjectA.RotationMatrix * simulationJoint.JointActDirection1;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (axisRotated).Normalize ();
			Vector3 t2 = Vector3.Cross (axisRotated, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Matrix3x3 skewP1 = Matrix3x3.GetSkewSymmetricMatrix (r1);
			Matrix3x3 skewP2 = Matrix3x3.GetSkewSymmetricMatrix (r2);

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 angularError = JacobianBuilderCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				simulationJoint);

			#endregion


			#region Jacobian Constraint

			//DOF 1

			double constraintLimit = simulationJoint.K * linearError.x;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF(
				indexA,
				indexB,
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (-skewP1.r1c1, -skewP1.r1c2, -skewP1.r1c3),
				new Vector3 (skewP2.r1c1,skewP2.r1c2,skewP2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = simulationJoint.K * linearError.y;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (-skewP1.r2c1, -skewP1.r2c2, -skewP1.r2c3),
				new Vector3 (skewP2.r2c1,skewP2.r2c2,skewP2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = simulationJoint.K * linearError.z;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (-skewP1.r3c1, -skewP1.r3c2, -skewP1.r3c3),
				new Vector3 (skewP2.r3c1,skewP2.r3c2,skewP2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			double angularLimit = simulationJoint.K *
				t1.Dot (angularError);

			hingeConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t1, 
					1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			//DOF 5

			angularLimit = simulationJoint.K *
				t2.Dot (angularError);

			hingeConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t2, 
					1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			#region Limit Constraints 

			//Limit extraction
			double angularLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMax);
			double angularLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMin);

			hingeConstraints.Add(JacobianBuilderCommon.GetAngularLimit (
				indexA, 
				indexB, 
				simulationJoint, 
				simulationObjectA, 
				simulationObjectB, 
				axisRotated,
				angularLimitMin,
				angularLimitMax));

			#endregion

			#endregion

			return hingeConstraints;
		}
	}
}

