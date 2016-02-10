using System;

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
			this.Value = value;
			this.Index = index;
			this.Count = value.Length;
		}

		#endregion
	}
}

