using System;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public struct ContactIndex
	{
		#region Public Properties

		public readonly int IndexA;
		public readonly int IndexB;
		public readonly ContactGroupType Type;

		#endregion

		#region Constructor

		public ContactIndex(
			int indexA,
			int indexB,
			ContactGroupType type)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.Type = type;
		}

		#endregion
	}
}

