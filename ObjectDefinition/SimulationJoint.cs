using System;
using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public struct SimulationJoint
	{
		#region public properties

		public readonly int IndexA;
		public readonly int IndexB;
		public readonly Joint[] JointList;

		#endregion

		#region Contructor

		public SimulationJoint (
			int indexA,
			int indexB,
			Joint[] jointList)
			:this()
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.JointList = jointList;
		}

		#endregion

	}
}

