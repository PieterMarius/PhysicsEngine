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
using static SharpEngineMathUtility.SparseVector;

namespace SharpEngineMathUtility
{
    public struct SparseMatrix
    {
        #region Fields

        public SparseVector[] Rows;
        public int n;
        public int m;

        #endregion

        #region Constructor

        public SparseMatrix(
            SparseVector[] rows,
            int nRows,
            int nColumn)
        {
            Rows = rows;
            n = nRows;
            m = nColumn;
        }

        public SparseMatrix(
            int nRows,
            int nColumn)
        {
            n = nRows;
            m = nColumn;
            Rows = new SparseVector[nRows];
        }

        #endregion

        #region Public Methods

        public static double[] Multiply(
            SparseMatrix matrix,
            double[] vector,
            int? maxThread = null)
        {
            if (matrix.n > 0)
            {
                double[] result = new double[matrix.n];

                var rangePartitioner = Partitioner.Create(0, matrix.n);
                ParallelOptions options = new ParallelOptions();

                if (maxThread.HasValue)
                {
                    options = new ParallelOptions() { MaxDegreeOfParallelism = maxThread.Value };
                    rangePartitioner = Partitioner.Create(0, matrix.n, Convert.ToInt32(matrix.n / maxThread.Value) + 1);
                }

                Parallel.ForEach(
                    rangePartitioner,
                    options,
                    (range, loopState) =>
                    {
                        for (int i = range.Item1; i < range.Item2; i++)
                        {
                            SparseVector m = matrix.Rows[i];

                            double[] bufValue = m.Value;
                            int[] bufIndex = m.Index;

                            double bValue = 0.0;

                            for (int j = 0; j < m.Index.Length; j++)
                                bValue += bufValue[j] * vector[bufIndex[j]];

                            result[i] = bValue;
                        }
                    });

                return result;
            }

            return null;
        }

        public static SparseMatrix Multiply(
            SparseMatrix matrixA,
            SparseMatrix matrixB,
            int? maxThread = null)
        {
            if (matrixA.m != matrixB.n)
                throw new Exception("Cannot multiply matrix");

            var result = new SparseMatrix(matrixA.n, matrixB.m);

            for (int i = 0; i < matrixB.m; i++)
            {
                var mul = Multiply(matrixA, GetColumn(matrixB, i));
                result.Rows[i] = GetSparseElement(mul);
            }

            return Transpose(result);
        }

        public static SparseMatrix Transpose(SparseMatrix matrix)
        {
            SparseMatrix result = new SparseMatrix(matrix.n, matrix.m);

            for (int i = 0; i < matrix.n; i++)
            {
                List<int> idx = new List<int>();
                List<double> val = new List<double>();

                for (int j = 0; j < matrix.m; j++)
                {
                    for (int w = 0; w < matrix.Rows[j].Index.Length; w++)
                    {
                        if (matrix.Rows[j].Index[w] == i)
                        {
                            idx.Add(j);
                            val.Add(matrix.Rows[j].Value[w]);
                        }
                    }
                }

                result.Rows[i] = new SparseVector(val.ToArray(), idx.ToArray(), matrix.m);
            }

            return result;
        }

        public static SparseMatrix Square(
            SparseMatrix matrix,
            int? maxThread = null)
        {
            var result = new SparseMatrix(matrix.n, matrix.m);

            for (int i = 0; i < matrix.n; i++)
            {
                var mul = Multiply(matrix, GetArray(matrix.Rows[i]));
                result.Rows[i] = GetSparseElement(mul);
            }

            return result;
        }

        public static double[] GetColumn(SparseMatrix mat, int c)
        {
            var res = new double[mat.n];

            for (int i = 0; i < mat.n; i++)
            {
                if (mat.Rows[i].Elements.TryGetValue(c, out double val))
                    res[i] = val;
                else
                    res[i] = 0.0;
            }
            return res;
        }

        public static void SetColumn(ref SparseMatrix mat, double[] vec, int idx)
        {
            for (int i = 0; i < mat.n; i++)
            {
                var sparseRow = mat.Rows[i].GetArray();
                sparseRow[idx] = vec[i];
                mat.Rows[i] = GetSparseElement(sparseRow);
            }
        }

        public static SparseVector GetSparseColumn(SparseMatrix mat, int c)
        {
            var values = new List<double>();
            var indexes = new List<int>();

            for (int i = 0; i < mat.n; i++)
            {
                if (mat.Rows[i].Elements.TryGetValue(c, out double val))
                {
                    values.Add(val);
                    indexes.Add(i);
                }
            }
            return new SparseVector(values.ToArray(), indexes.ToArray(), mat.m);
        }

        public static SparseMatrix GetSparseIdentityMatrix(int n, int m, double value)
        {
            var res = new SparseMatrix(n, m);

            for (int i = 0; i < n; i++)
            {
                var idx = new int[] { i };
                var val = new double[] { value };
                res.Rows[i] = new SparseVector(val, idx, m);
            }

            return res;
        }

        #endregion

    }
}
