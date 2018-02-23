using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace SharpEngineMathUtility
{
	public struct SparseElement: IEqualityComparer<SparseElement>
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

        #region Public Methods

        public static double[] Multiply(SparseElement[] matrix, double[] vector)
        {
            double[] result = new double[matrix.Length];

            Parallel.For(0,
                matrix.Length,
                i =>
                {
                    SparseElement m = matrix[i];

                    double[] bufValue = m.Value;
                    int[] bufIndex = m.Index;

                    double bValue = 0.0;

                    for (int j = 0; j < m.Count; j++)
                        bValue += bufValue[j] * vector[bufIndex[j]];

                    result[i] = bValue;
                });

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

