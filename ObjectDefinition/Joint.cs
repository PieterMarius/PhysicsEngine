using System;
using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public struct Joint
	{
		#region Public Fields

		public readonly double K;
		public readonly double C;
		public readonly ConstraintType Type;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 AnchorPoint;
		public readonly Vector3 DistanceFromA;
		public readonly Vector3 DistanceFromB;
		public readonly Quaternion RelativeOrientation;
		public readonly Vector3 RotationAxis;
		public readonly Vector3 TranslationAxis;

		#endregion

		#region Constructor

		public Joint(
			double K,
			double C,
			ConstraintType type,
			Vector3 startAnchorPoint,
			Vector3 anchorPoint,
			Vector3 distanceFromA,
			Vector3 distanceFromB,
			Quaternion relativeOrientation,
			Vector3 rotationAxis,
			Vector3 translationAxis)
		{
			this.K = K;
			this.C = C;
			this.Type = type;
			this.StartAnchorPoint = startAnchorPoint;
			this.AnchorPoint = anchorPoint;
			this.DistanceFromA = distanceFromA;
			this.DistanceFromB = distanceFromB;
			this.RelativeOrientation = relativeOrientation;
			this.RotationAxis = rotationAxis;
			this.TranslationAxis = translationAxis;
		}

		#endregion
	}
}

