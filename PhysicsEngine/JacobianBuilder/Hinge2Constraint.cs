using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public static class Hinge2Constraint
	{
		#region Public Methods

		/// <summary>
		/// Sets the hinge2 joint.
		/// </summary>
		/// <returns>The hinge2 joint.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="startAnchorPosition">Start anchor position.</param>
		/// <param name="hingeAxis">Hinge axis.</param>
		/// <param name="rotationAxis">Rotation axis.</param>
		/// <param name="K">K.</param>
		/// <param name="C">C.</param>
		/// <param name="angularLimitMin">Angular limit minimum.</param>
		/// <param name="angularLimitMax">Angular limit max.</param>
		public static Joint SetJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
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

			hingeAxis = hingeAxis.Normalize ();
			rotationAxis = rotationAxis.Normalize ();

			Vector3 rHingeAxis = objectA.RotationMatrix * hingeAxis;
			Vector3 rRotationAxis = objectB.RotationMatrix * rotationAxis;

			Quaternion relativeOrientation1 = calculateRelativeRotation (
				                                  rHingeAxis,
				                                  rRotationAxis,
				                                  objectA.RotationStatus);

			Quaternion relativeOrientation2 = calculateRelativeRotation (
				                                  rRotationAxis,
				                                  rHingeAxis,
				                                  objectB.RotationStatus);

			Vector3 angularLimitMinVec = new Vector3 ();
			Vector3 angularLimitMaxVec = new Vector3 ();

			if (angularLimitMin != angularLimitMax) {
				angularLimitMinVec = hingeAxis * angularLimitMin;
				angularLimitMaxVec = hingeAxis * angularLimitMax;
			}

			Joint joint = new Joint (
				              K,
				              C,
				              JointType.Hinge2,
				              startAnchorPosition,
				              anchorPosition,
				              distanceFromA,
				              distanceFromB,
				              relativeOrientation1,
				              relativeOrientation2,
				              hingeAxis,
				              rotationAxis,
				              new Vector3 (),
				              new Vector3 (),
				              angularLimitMinVec,
				              angularLimitMaxVec);

			return joint;
		}

		/// <summary>
		/// Builds the hinge2 joint.
		/// </summary>
		/// <returns>The hinge2 joint.</returns>
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

			Vector3 hingeAxis = simulationObjectA.RotationMatrix * simulationJoint.JointActDirection1;

			Vector3 rotationAxis = simulationObjectB.RotationMatrix * simulationJoint.JointActDirection2;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3 t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

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

			double angularLimit = simulationJoint.K * (-k);

			hingeConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			#region Limit Constraints 

			//Limit extraction
			double angularLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMax);
			double angularLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMin);

			double angle1 = getAngle1(
				hingeAxis,
				rotationAxis,
				simulationJoint.JointActDirection1,
				simulationObjectA.RotationStatus,
				simulationJoint.RelativeRotation1);

			double angle2 = getAngle2 (
				hingeAxis,
				rotationAxis,
				simulationJoint.JointActDirection2,
				simulationObjectB.RotationStatus,
				simulationJoint.RelativeRotation2);
				
			Console.WriteLine ("angle1 " + angle1);
			Console.WriteLine ("angle2 " + angle2);

			//			hingeConstraints.Add(JacobianBuilderCommon.GetAngularLimit (
			//				indexA, 
			//				indexB, 
			//				simulationJoint, 
			//				simulationObjectA, 
			//				simulationObjectB, 
			//				hingeAxis,
			//				angularLimitMin,
			//				angularLimitMax));

			#endregion

			#endregion

			return hingeConstraints;
		}

		#endregion

		#region Private Methods

		//TODO verificare se è possibile fondere in un unico metodo
		private static double getAngle1(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix (axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion (rotationMatrix);

			Quaternion mult1 = Quaternion.Multiply1 (rotationStatus, rotationQ);
			Quaternion mult2 = Quaternion.Multiply2 (mult1, startRelativeRotation);

			Vector3 quaternionVectorPart = new Vector3 (
				mult2.b,
				mult2.c,
				mult2.d);

			return JacobianBuilderCommon.GetRotationAngle (quaternionVectorPart, mult2.a, startAxis);
		}

		private static double getAngle2(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix (axis2, axis1);
			Quaternion rotationQ = Quaternion.GetQuaternion (rotationMatrix);

			Quaternion mult1 = Quaternion.Multiply1 (rotationStatus, rotationQ);
			Quaternion mult2 = Quaternion.Multiply2 (mult1, startRelativeRotation);

			Vector3 quaternionVectorPart = new Vector3 (
				mult2.b,
				mult2.c,
				mult2.d);

			return - JacobianBuilderCommon.GetRotationAngle (quaternionVectorPart, mult2.a, startAxis);
		}

		private static Quaternion calculateRelativeRotation(
			Vector3 axis1,
			Vector3 axis2,
			Quaternion bodyRotationStatus)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix (axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion (rotationMatrix);

			return Quaternion.Multiply1 (bodyRotationStatus, rotationQ);
		}

		#endregion
	}
}

