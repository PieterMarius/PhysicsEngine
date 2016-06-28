
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
			IndexA = indexA;
			IndexB = indexB;
			Type = type;
		}

		#endregion
	}
}

