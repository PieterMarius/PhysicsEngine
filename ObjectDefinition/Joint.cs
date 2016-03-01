using System;
using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public struct Joint
	{
		public readonly double K;
		public readonly double C;
		public readonly ConstraintType Type;
		public readonly Vector3 StartJointPos;
		public readonly Vector3 DistanceFromA;
		public readonly Vector3 DistanceFromB;
		public readonly Vector3 RotationAxis;
		//public readonly Vector3 RotationConstraintB;
		public readonly Vector3 TranslationAxis;
		public readonly Vector3 Position;

		public Joint(
			double K,
			double C,
			ConstraintType type,
			Vector3 startJointPos,
			Vector3 position,
			Vector3 distanceFromA,
			Vector3 distanceFromB,
			Vector3 rotationAxis,
			Vector3 translationAxis)
		{
			this.K = K;
			this.C = C;
			this.Type = type;
			this.StartJointPos = startJointPos;
			this.Position = position;
			this.DistanceFromA = distanceFromA;
			this.DistanceFromB = distanceFromB;
			this.RotationAxis = rotationAxis;
			this.TranslationAxis = translationAxis;
		}
	}
}

