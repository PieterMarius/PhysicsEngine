
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

        public static double[] Multiply(SparseElement[] matrix, double[] vector)
        {
            if (matrix[0].RowLength != vector.Length)
                throw new Exception("Wrong input length");

            double[] result = new double[matrix.Length];

            for (int i = 0; i < matrix.Length; i++)
            {
                SparseElement m = matrix[i];

                double[] bufValue = m.Value;
                int[] bufIndex = m.Index;

                double bValue = 0.0;

                for (int j = 0; j < m.Count; j++)
                    bValue += bufValue[j] * vector[bufIndex[j]];

                result[i] = bValue;
            }

            return result;
        }

        public bool Equals(SparseElement x, SparseElement y)
        {
            if (x.Count != y.Count)
                return false;

            if (x.Index.Length != x.Index.Length)
                return false;

            if (x.Value.Length != x.Value.Length)
                return false;
                        
            return true;
        }

        public int GetHashCode(SparseElement obj)
        {
            throw new NotImplementedException();
        }

        #endregion 


    }
}

