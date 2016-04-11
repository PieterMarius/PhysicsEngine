using System;

namespace CollisionEngine
{
	public struct CollisionPair
	{
		public readonly int objectIndexA;
		public readonly int objectIndexB;

		public CollisionPair(
			int indexA,
			int indexB)
		{
			this.objectIndexA = indexA;
			this.objectIndexB = indexB;
		}
	}
}

