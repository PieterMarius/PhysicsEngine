using System;
using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public struct Joint
	{
		#region Public Fields

		public readonly double K;
		public readonly double C;
		public readonly JointType Type;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 AnchorPoint;
		public readonly Vector3 StartErrorAxis1;
		public readonly Vector3 StartErrorAxis2;
		public readonly Quaternion RelativeOrientation;
		public readonly Vector3 JointActDirection;
		public readonly Vector3 LinearLimitMin;
		public readonly Vector3 LinearLimitMax;
		public readonly Vector3 AngularLimitMin;
		public readonly Vector3 AngularLimitMax;

		#endregion

		#region Constructor

		public Joint(
			double K,
			double C,
			JointType type,
			Vector3 startAnchorPoint,
			Vector3 anchorPoint,
			Vector3 startErrorAxis1,
			Vector3 startErrorAxis2,
			Quaternion relativeOrientation,
			Vector3 jointActDirection,
			Vector3 linearLimitMin,
			Vector3 linearLimitMax,
			Vector3 angularLimitMin,
			Vector3 angularLimitMax)
			:this()
		{
			this.K = K;
			this.C = C;
			this.Type = type;
			this.StartAnchorPoint = startAnchorPoint;
			this.AnchorPoint = anchorPoint;
			this.StartErrorAxis1 = startErrorAxis1;
			this.StartErrorAxis2 = startErrorAxis2;
			this.RelativeOrientation = relativeOrientation;
			this.JointActDirection = jointActDirection;
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;
			this.AngularLimitMin = angularLimitMin;
			this.AngularLimitMax = angularLimitMax;
		}

		#endregion

		#region Public Methods

		/// <summary>
		/// Sets the fixed joint.
		/// </summary>
		/// <returns>The fixed joint.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="K">K.</param>
		/// <param name="C">C.</param>
		public static Joint SetFixedJoint(
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
				              Vector3.ToZero (),
				              new Vector3 (),
				              new Vector3 (),
				              new Vector3 (),
				              new Vector3 ());

			return joint;
		}

		/// <summary>
		/// Sets the slider joint.
		/// </summary>
		/// <returns>The slider joint.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="K">K.</param>
		/// <param name="C">C.</param>
		/// <param name="linearLimitMin">Linear limit minimum.</param>
		/// <param name="linearLimitMax">Linear limit max.</param>
		public static Joint SetSliderJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 sliderAxis,
			double K,
			double C,
			double linearLimitMin = 0.0,
			double linearLimitMax = 0.0)
		{
			sliderAxis = -1.0 * sliderAxis.Normalize ();

			Vector3 relativePos = objectA.RotationMatrix *
			                      (startAnchorPosition - objectA.StartPosition);

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
			                        (anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
			                        (anchorPosition - objectB.Position);
			
			Quaternion relativeOrientation = objectB.RotationStatus.Inverse () *
			                                 objectA.RotationStatus;

			Vector3 linearLimitMinVec = sliderAxis * linearLimitMin;
			Vector3 linearLimitMaxVec = sliderAxis * linearLimitMax;

			Joint joint = new Joint (
				              K,
				              C,
				              JointType.Slider,
				              startAnchorPosition,
				              anchorPosition,
				              distanceFromA,
				              distanceFromB,
				              relativeOrientation,
				              sliderAxis,
				              linearLimitMinVec,
				              linearLimitMaxVec,
				              new Vector3 (),
				              new Vector3 ());

			return joint;
		}

		/// <summary>
		/// Sets the ball socket joint.
		/// </summary>
		/// <returns>The ball socket joint.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="K">K.</param>
		/// <param name="C">C.</param>
		/// <param name="startAnchorPosition">Start anchor position (default: objectB.Position - objectA.Position * 0.5).</param>
		public static Joint SetBallSocketJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			double K,
			double C)
		{
			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
				(anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
				(anchorPosition - objectB.Position);

			Joint joint = new Joint (
				              K,
				              C,
				              JointType.BallAndSocket,
				              startAnchorPosition,
				              anchorPosition,
				              distanceFromA,
				              distanceFromB,
				              new Quaternion (),
				              Vector3.ToZero (),
				              new Vector3 (),
				              new Vector3 (),
				              new Vector3 (),
				              new Vector3 ());

			return joint;
		}

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
		public static Joint SetPistonJoint(
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
				              pistonAxis,
				              linearLimitMinVec,
				              linearLimitMaxVec,
				              angularLimitMinVec,
				              angularLimitMaxVec);

			return joint;
		}

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
		public static Joint SetHingeJoint(
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
				              hingeAxis,
				              new Vector3 (),
				              new Vector3 (),
				              angularLimitMinVec,
				              angularLimitMaxVec);

			return joint;
		}

		public static Joint[] SetHinge2Joint(
			SimulationObject objectA,
			SimulationObject objectB,
			SimulationObject connector,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double K,
			double C,
			double angularLimitMin = 0.0,
			double angularLimitMax = 0.0,
			Vector3 startAnchorPosition = new Vector3 ())
		{
			Joint[] hinge2 = new Joint[2];

			if (startAnchorPosition.Length () == 0.0)
				startAnchorPosition = (objectB.Position - objectA.Position) * 0.5;

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
				(anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
				(anchorPosition - objectB.Position);

			Quaternion relativeOrientation = objectA.RotationStatus.Inverse () *
				objectB.RotationStatus;

			hingeAxis = hingeAxis.Normalize ();
			rotationAxis = rotationAxis.Normalize ();

			Vector3 angularLimitMinVec = hingeAxis * angularLimitMin;
			Vector3 angularLimitMaxVec = hingeAxis * angularLimitMax;

			hinge2[0] = new Joint (
				K,
				C,
				JointType.Hinge,
				startAnchorPosition,
				anchorPosition,
				distanceFromA,
				distanceFromB,
				relativeOrientation,
				hingeAxis,
				new Vector3 (),
				new Vector3 (),
				angularLimitMinVec,
				angularLimitMaxVec);

			hinge2 [1] = new Joint (
				K,
				C,
				JointType.Hinge,
				startAnchorPosition,
				anchorPosition,
				distanceFromA,
				distanceFromB,
				relativeOrientation,
				rotationAxis,
				new Vector3 (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 ());

			return hinge2;
		}

		public static Joint Set6DOFJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			double K,
			double C,
			Vector3 linearLimitMin,
			Vector3 linearLimitMax,
			Vector3 angularLimitMin,
			Vector3 angularLimitMax,
			Vector3 startAnchorPosition = new Vector3 ())
		{
			if (startAnchorPosition.Length () == 0.0)
				startAnchorPosition = (objectB.Position - objectA.Position) * 0.5;

			Vector3 relativePos = objectA.RotationMatrix *
			                      (startAnchorPosition - objectA.StartPosition);

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
			                        (anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
			                        (anchorPosition - objectB.Position);

			Quaternion relativeOrientation = objectA.RotationStatus.Inverse () *
			                                 objectB.RotationStatus;

			Joint joint = new Joint (
				              K,
				              C,
				              JointType.Generic6DOF,
				              startAnchorPosition,
				              anchorPosition,
				              distanceFromA,
				              distanceFromB,
				              relativeOrientation,
				              new Vector3 (),
				              linearLimitMin,
				              linearLimitMax,
				              angularLimitMin,
				              angularLimitMax);

			return joint;
		}

		#endregion

	}
}

