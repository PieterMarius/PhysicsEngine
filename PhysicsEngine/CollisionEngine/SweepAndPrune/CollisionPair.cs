
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
			objectIndexA = indexA;
			objectIndexB = indexB;
		}
	}
}

