/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using System;
using System.Collections.Concurrent;
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

        public static double[] Multiply(
            SparseElement[] matrix, 
            double[] vector,
            int? maxThread = null)
        {
            double[] result = new double[matrix.Length];

            var rangePartitioner = Partitioner.Create(0, matrix.Length);
            ParallelOptions options = new ParallelOptions();

            if (maxThread.HasValue)
            {
                options = new ParallelOptions() { MaxDegreeOfParallelism = maxThread.Value };
                rangePartitioner = Partitioner.Create(0, matrix.Length, Convert.ToInt32(matrix.Length / maxThread.Value) + 1);
            }
            
            Parallel.ForEach(
                rangePartitioner,
                options,
                (range, loopState) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        SparseElement m = matrix[i];

                        double[] bufValue = m.Value;
                        int[] bufIndex = m.Index;

                        double bValue = 0.0;

                        for (int j = 0; j < m.Count; j++)
                            bValue += bufValue[j] * vector[bufIndex[j]];

                        result[i] = bValue;
                    }
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

