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
		public readonly Vector3 Axis3;
		public readonly double LinearLimitMin;
		public readonly double LinearLimitMax;
		public readonly double AngularLimitMin;
		public readonly double AngularLimitMax;

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
			Vector3 axis3,
			double linearLimitMin,
			double linearLimitMax,
			double angularLimitMin,
			double angularLimitMax)
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
			this.Axis3 = axis3;
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;
			this.AngularLimitMin = angularLimitMin;
			this.AngularLimitMax = angularLimitMax;
		}

		#endregion

		#region Public Methods

		public static Joint SetFixedJoint(
			Vector3 startAnchorPosition,
			SimulationObject objectA,
			SimulationObject objectB,
			double K,
			double C)
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
				              Vector3.ToZero (),
				              0,
				              0,
				              0,
				              0);

			return joint;
		}

		public static Joint SetSliderJoint(
			Vector3 startAnchorPosition,
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 sliderAxis,
			double K,
			double C,
			double linearLimitMin = 0.0,
			double linearLimitMax = 0.0)
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

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (sliderAxis);

			Vector3 t2 = Vector3.Cross (sliderAxis, t1);

			Joint joint = new Joint (
				K,
				C,
				JointType.Slider,
				startAnchorPosition,
				anchorPosition,
				distanceFromA,
				distanceFromB,
				relativeOrientation,
				t1,
				t2,
				sliderAxis,
				linearLimitMin,
				linearLimitMax,
				0,
				0);

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

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (hingeAxis);

			Vector3 t2 = Vector3.Cross (hingeAxis, t1);

			Joint joint = new Joint (
				K,
				C,
				JointType.Hinge,
				startAnchorPosition,
				anchorPosition,
				distanceFromA,
				distanceFromB,
				relativeOrientation,
				t1,
				t2,
				hingeAxis,
				0,
				0,
				angularLimitMin,
				angularLimitMax);

			return joint;
		}

		#endregion

	}
}

