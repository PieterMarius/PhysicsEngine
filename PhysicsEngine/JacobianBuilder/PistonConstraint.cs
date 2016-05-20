using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public static class PistonConstraint
	{
		/// <summary>
		/// Sets the piston joint.
		/// </summary>
		/// <returns>The piston joint.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="K">K.</param>
		/// <param name="C">C.</param>
		/// <param name="linearLimitMin">Linear limit minimum.</param>
		/// <param name="linearLimitMax">Linear limit max.</param>
		/// <param name="angularLimitMin">Angular limit minimum.</param>
		/// <param name="angularLimitMax">Angular limit max.</param>
		public static Joint SetJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 pistonAxis,
			double K,
			double C,
			double linearLimitMin = 0.0,
			double linearLimitMax = 0.0,
			double angularLimitMin = 0.0,
			double angularLimitMax = 0.0)
		{
			pistonAxis = -1.0 * pistonAxis.Normalize ();

			Vector3 relativePos = objectA.RotationMatrix *
				(startAnchorPosition - objectA.StartPosition);

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
				(anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
				(anchorPosition - objectB.Position);

			Quaternion relativeOrientation = objectB.RotationStatus.Inverse () *
				objectA.RotationStatus;

			Vector3 linearLimitMinVec = pistonAxis * linearLimitMin;
			Vector3 linearLimitMaxVec = pistonAxis * linearLimitMax;

			Vector3 angularLimitMinVec = pistonAxis * angularLimitMin;
			Vector3 angularLimitMaxVec = pistonAxis * angularLimitMax;

			Joint joint = new Joint (
				K,
				C,
				JointType.Piston,
				startAnchorPosition,
				anchorPosition,
				distanceFromA,
				distanceFromB,
				relativeOrientation,
				new Quaternion (),
				pistonAxis,
				new Vector3 (),
				linearLimitMinVec,
				linearLimitMaxVec,
				angularLimitMinVec,
				angularLimitMaxVec);

			return joint;
		}

		/// <summary>
		/// Builds the piston joint.
		/// </summary>
		/// <returns>The piston joint.</returns>
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
			List<JacobianContact> pistonConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * simulationJoint.JointActDirection1;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3 t2 = Vector3.Cross (sliderAxis, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			Vector3 angularError = sliderAxis.Cross (
				(simulationObjectB.RotationMatrix * simulationJoint.JointActDirection1));

			#region Jacobian Constraint

			#region Constraints

			//DOF 1

			double angularLimit = simulationJoint.K *
				t1.Dot (angularError);

			pistonConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					1.0 * t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			//DOF 2

			angularLimit = simulationJoint.K *
				t2.Dot (angularError);

			pistonConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					1.0 * t2, 
					-1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			//DOF 3

			double constraintLimit = simulationJoint.K * Vector3.Dot (t1,linearError);

			Console.WriteLine ("linear error:" + constraintLimit);

			pistonConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = simulationJoint.K * Vector3.Dot (t2,linearError);

			Console.WriteLine ("linear error:" + constraintLimit);

			pistonConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				t2,
				-1.0 * t2,
				Vector3.Cross (r1, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			// Limit extraction
			double linearLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.LinearLimitMin);
			double linearLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.LinearLimitMax);

			pistonConstraints.Add (
				JacobianBuilderCommon.GetLinearLimit(
					indexA,
					indexB,
					simulationJoint,
					simulationObjectA,
					simulationObjectB,
					sliderAxis,
					r1,
					r2,
					linearLimitMin,
					linearLimitMax));

			// Limit extraction
			double angularLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMin);
			double angularLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMax);

			pistonConstraints.Add(
				JacobianBuilderCommon.GetAngularLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB, 
					sliderAxis,
					angularLimitMin,
					angularLimitMax));

			#endregion

			#endregion

			return pistonConstraints;
		}
	}
}

