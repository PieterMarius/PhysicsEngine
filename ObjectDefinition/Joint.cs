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
		public readonly Quaternion RelativeRotation1;
		public readonly Quaternion RelativeRotation2;
		public readonly Vector3 JointActDirection1;
		public readonly Vector3 JointActDirection2;
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
			Quaternion relativeRotation1,
			Quaternion relativeRotation2,
			Vector3 jointActDirection1,
			Vector3 jointActDirection2,
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
			this.RelativeRotation1 = relativeRotation1;
			this.RelativeRotation2 = relativeRotation2;
			this.JointActDirection1 = jointActDirection1;
			this.JointActDirection2 = jointActDirection2;
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;
			this.AngularLimitMin = angularLimitMin;
			this.AngularLimitMax = angularLimitMax;
		}

		#endregion

		#region Public Methods

		//TODO da eliminare
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
				              new Quaternion (),
				              new Vector3 (),
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

