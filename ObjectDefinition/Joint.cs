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
		public readonly Vector3 DistanceFromA;
		public readonly Vector3 DistanceFromB;
		public readonly Quaternion RelativeOrientation;
		public readonly Vector3 Axis1;
		public readonly Vector3 Axis2;
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
			Vector3 distanceFromA,
			Vector3 distanceFromB,
			Quaternion relativeOrientation,
			Vector3 axis1,
			Vector3 axis2,
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
			this.DistanceFromA = distanceFromA;
			this.DistanceFromB = distanceFromB;
			this.RelativeOrientation = relativeOrientation;
			this.Axis1 = axis1;
			this.Axis2 = axis2;
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;
			this.AngularLimitMin = angularLimitMin;
			this.AngularLimitMax = angularLimitMax;
		}

		#endregion

		#region Public Methods

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
				              Vector3.ToZero (),
				              new Vector3 (),
				              new Vector3 (),
				              new Vector3 (),
				              new Vector3 ());

			return joint;
		}

		public static Joint SetSliderJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			double K,
			double C,
			double linearLimitMin = 0.0,
			double linearLimitMax = 0.0)
		{
			Vector3 startAnchorPosition = (objectB.Position - objectA.Position) * 0.5;

			Vector3 sliderAxis = -1.0 * startAnchorPosition.Normalize ();

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
				(anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
				(anchorPosition - objectB.Position);

			Quaternion relativeOrientation = Quaternion.Inverse (objectA.RotationStatus) *
				objectB.RotationStatus;

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
				              new Vector3 (),
				              linearLimitMinVec,
				              linearLimitMaxVec,
				              new Vector3 (),
				              new Vector3 ());

			return joint;
		}

		public static Joint SetBallSocketJoint(
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

			Joint joint = new Joint (
				K,
				C,
				JointType.BallAndSocket,
				startAnchorPosition,
				anchorPosition,
				distanceFromA,
				distanceFromB,
				new Quaternion(),
				Vector3.ToZero (),
				Vector3.ToZero (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 ());

			return joint;
		}

		public static Joint SetPistonJoint(
			SimulationObject objectA,
			SimulationObject objectB,
			double K,
			double C,
			double linearLimitMin = 0.0,
			double linearLimitMax = 0.0)
		{
			Vector3 startAnchorPosition = (objectB.Position - objectA.Position) * 0.5;

			Vector3 pistonAxis = -1.0 * startAnchorPosition.Normalize ();

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = objectA.RotationMatrix.Transpose () *
				(anchorPosition - objectA.Position);

			Vector3 distanceFromB = objectB.RotationMatrix.Transpose () *
				(anchorPosition - objectB.Position);

			Quaternion relativeOrientation = Quaternion.Inverse (objectA.RotationStatus) *
				objectB.RotationStatus;

			Vector3 linearLimitMinVec = pistonAxis * linearLimitMin;
			Vector3 linearLimitMaxVec = pistonAxis * linearLimitMax;

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
				              new Vector3 (),
				              linearLimitMinVec,
				              linearLimitMaxVec,
				              new Vector3 (),
				              new Vector3 ());

			return joint;
		}

		public static Joint SetHingeJoint(
			Vector3 startAnchorPosition,
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 hingeAxis,
			double K,
			double C,
			double angularLimitMin = 0.0,
			double angularLimitMax = 0.0)
		{
			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			Vector3 anchorPosition = relativePos + objectA.Position;

			Vector3 distanceFromA = Matrix3x3.Transpose (objectA.RotationMatrix) *
				(anchorPosition - objectA.Position);

			Vector3 distanceFromB = Matrix3x3.Transpose (objectB.RotationMatrix) *
				(anchorPosition - objectB.Position);

			Quaternion relativeOrientation = Quaternion.Inverse (objectA.RotationStatus) *
				objectB.RotationStatus;

			hingeAxis = hingeAxis.Normalize ();

			Vector3 angularLimitMinVec = hingeAxis * angularLimitMin;
			Vector3 angularLimitMaxVec = hingeAxis * angularLimitMax;

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
				              new Vector3 (),
				              angularLimitMinVec,
				              angularLimitMaxVec);

			return joint;
		}

		#endregion

	}
}

