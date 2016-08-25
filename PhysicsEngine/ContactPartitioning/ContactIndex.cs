
namespace MonoPhysicsEngine
{
	public struct ContactIndex
	{
		#region Public Properties

		public readonly int IndexA;
		public readonly int IndexB;
		public readonly ContactGroupType Type;
		public readonly int KeyIndex;

		#endregion

		#region Constructor

		public ContactIndex(
			int indexA,
			int indexB,
			ContactGroupType type,
			int keyIndex)
		{
			IndexA = indexA;
			IndexB = indexB;
			Type = type;
			KeyIndex = keyIndex;
		}

		#endregion
	}
}

