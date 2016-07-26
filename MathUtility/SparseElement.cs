
namespace PhysicsEngineMathUtility
{
	public struct SparseElement
	{
		#region Public fields

		public readonly double[] Value;
		public readonly int[] Index;
		public readonly int Count;

		#endregion

		#region Constructor

		public SparseElement (
			double[] value,
			int[] index)
		{
			Value = value;
			Index = index;
			Count = value.Length;
		}

		#endregion
	}
}

