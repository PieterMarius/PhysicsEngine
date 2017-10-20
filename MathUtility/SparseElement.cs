
using System;
using System.Collections.Generic;

namespace SharpEngineMathUtility
{
	public struct SparseElement: IEqualityComparer<SparseElement>
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

        #region Public Methods

        public bool Equals(SparseElement x, SparseElement y)
        {
            if (x.Count != y.Count)
                return false;

            if (x.Index.Length != x.Index.Length)
                return false;

            if (x.Value.Length != x.Value.Length)
                return false;

            //for (int i = 0; i < x.Index.Length; i++)
            //{
            //    if (x.Index[i] != y.Index[i])
            //        return false;
            //}

            //for (int i = 0; i < x.Value.Length; i++)
            //{
            //    if (!x.Value[i].Equals(y.Value[i]))
            //        return false;
            //}

            return true;
        }

        public int GetHashCode(SparseElement obj)
        {
            throw new NotImplementedException();
        }

        #endregion 


    }
}

