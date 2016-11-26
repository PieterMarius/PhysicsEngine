
namespace PhysicsEngineMathUtility
{
	public struct SparseElement
	{
		#region Public fields

		public readonly double[] Value;
		public readonly int[] Index;
		public readonly int Count;
        public readonly int RowLength;

		#endregion

		#region Constructor

		public SparseElement (
			double[] value,
			int[] index,
            int rowLength)
		{
			Value = value;
			Index = index;
            RowLength = rowLength;
            Count = value.Length;
		}

		#endregion
	}
}

