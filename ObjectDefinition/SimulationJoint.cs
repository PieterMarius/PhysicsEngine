using System;
using PhysicsEngineMathUtility;

namespace ObjectDefinition
{
	public struct SimulationJoint
	{
		#region public properties

		public readonly int IndexA;
		public readonly int IndexB;
		public readonly double K;
		public readonly double C;
		public readonly Vector3 StartJointPos;
		public readonly Vector3 Axis1;
		public readonly Vector3 Axis2;
		public readonly Vector3 Axis3;
		public readonly Vector3 DistanceFromA;
		public readonly Vector3 DistanceFromB;
		public readonly Vector3 RotationConstraintA;
		public readonly Vector3 RotationConstraintB;
		public readonly Vector3 Position;

		#endregion

		#region Contructor

		public SimulationJoint (
			int indexA,
			int indexB,
			double K,
			double C,
			Vector3 startJointPos,
			Vector3 position,
			Vector3 axis1,
			Vector3 axis2,
			Vector3 axis3,
			Vector3 distanceFromA,
			Vector3 distanceFromB,
			Vector3 rotationConstraintA,
			Vector3 rotationConstraintB)
			:this()
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.K = K;
			this.C = C;
			this.StartJointPos = startJointPos;
			this.Position = position;
			this.Axis1 = axis1;
			this.Axis2 = axis2;
			this.Axis3 = axis3;
			this.DistanceFromA = distanceFromA;
			this.DistanceFromB = distanceFromB;
			this.RotationConstraintA = rotationConstraintA;
			this.RotationConstraintB = rotationConstraintB;
		}

		#endregion

	}
}

