using System;

namespace MonoPhysicsEngine
{
	public struct ContactIndex
	{
		public readonly int IndexA;
		public readonly int IndexB;
		public readonly ContactType Type;

		public ContactIndex(
			int indexA,
			int indexB,
			ContactType type)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.Type = type;
		}
	}
}

