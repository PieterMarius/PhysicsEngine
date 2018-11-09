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
using System.Collections.Generic;
using static SharpEngineMathUtility.MathUtils;
using static SharpEngineMathUtility.SparseMatrix;

namespace SharpEngineMathUtility.Solver
{
    public sealed class HouseholderQR
    {
        #region Fields
                
        #endregion


        #region Constructor

        public HouseholderQR()
        { }

        #endregion

        #region Public Methods

        public double[] Solve(
            SparseMatrix A,
            double[] b)
        {
            QRDecomposition decomposition = Householder(A);

            var Qt = Transpose(decomposition.Q);
            var Qb = Multiply(Qt, b);

            return BackwardSubstitution(decomposition.R, Qb);
        }

        public QRDecomposition Householder(SparseMatrix mat)
        {
            int m = mat.n;
            int n = mat.m;

            List<SparseMatrix> qv = new List<SparseMatrix>();

            SparseMatrix z = new SparseMatrix(mat.n, mat.m);
            Array.Copy(mat.Rows, z.Rows, mat.n);

            SparseMatrix z1 = new SparseMatrix();

            for (int k = 0; k < n && k < m - 1; ++k)
            {
                double[] e = new double[m];
                double[] x = new double[m];

                z1 = ComputeMinor(z, k);
                x = GetColumn(z1, k);

                double a = Module(x);
                
                if (!mat.Rows[k].Elements.TryGetValue(k, out double val))
                    val = 0.0;

                if (val > 0.0)
                    a = -a;

                for (int i = 0; i < e.Length; i++)
                    e[i] = (i == k) ? 1.0 : 0.0;

                e = Add(x, Multiply(a, e));

                var mod = Module(e);

                if (mod != 0.0)
                    e = Multiply(1.0 / mod, e);
                
                qv.Add(ComputeHouseholderFactor(e));

                z = Multiply(qv[qv.Count - 1], z1);
            }

            var Q = qv[0];

            for (int i = 1; i < n && i < m - 1; ++i)
            {
                z1 = Multiply(qv[i], Q);
                Q = z1;
            }
            
            var R = Multiply(Q, mat);

            Q = Transpose(Q);

            return new QRDecomposition() { Q = Q, R = R };
        }

        public double[] LUSolve(SparseMatrix A, double[] b)
        {
            double[,] lu = new double[A.n, A.n];
            double sum = 0.0;

            for (int i = 0; i < A.n; i++)
            {
                for (int j = i; j < A.n; j++)
                {
                    sum = 0.0;
                    for (int k = 0; k < i; k++)
                        sum += lu[i, k] * lu[k, j];

                    if (A.Rows[i].Elements.TryGetValue(j, out double val))
                        lu[i, j] = val - sum;
                    else
                        lu[i, j] = -sum;
                }

                for (int j = i + 1; j < A.n; j++)
                {
                    sum = 0.0;
                    for (int k = 0; k < i; k++)
                        sum += lu[j, k] * lu[k, i];

                    double v = 0.0;
                    if (lu[i, i] != 0.0)
                        v = 1.0 / lu[i, i];

                    if (A.Rows[j].Elements.TryGetValue(i, out double val))
                        lu[j, i] = v * (val - sum);
                    else
                        lu[j, i] = v * (-sum);
                }
            }

            double[] y = new double[A.n];
            for (int i = 0; i < A.n; i++)
            {
                sum = 0.0;
                for (int k = 0; k < i; k++)
                    sum += lu[i, k] * y[k];
                y[i] = b[i] - sum;
            }

            double[] x = new double[A.n];
            for (int i = A.n - 1; i >= 0; i--)
            {
                sum = 0.0;
                for (int k = i + 1; k < A.n; k++)
                    sum += lu[i, k] * x[k];

                double v = 0.0;
                if (lu[i, i] != 0.0)
                    v = 1.0 / lu[i, i];

                x[i] = v * (y[i] - sum);
            }

            return x;

        }

        #endregion

        #region Private Methods

        private double[] BackwardSubstitution(SparseMatrix matrix, double[] vec)
        {
            int rows = vec.Length;
            double[] res = new double[rows];

            if(matrix.Rows[rows - 1].Elements.TryGetValue(rows-1, out double val))
                res[rows - 1] = vec[rows - 1] / val;
            
            for (int i = rows-2; i >= 0; i--)
            {
                res[i] = vec[i];
                var row = matrix.Rows[i].Elements;

                for (int j = i + 1; j < rows; ++j)
                {
                    if (row.TryGetValue(j, out double value))
                        res[i] -= value * res[j];
                }
                if (row.TryGetValue(i, out double value1))
                    res[i] = res[i] / value1;
            }

            return res;

        }

        private SparseMatrix ComputeMinor(SparseMatrix mat, int d)
        {
            var res = new SparseMatrix(mat.n, mat.m);

            for (int i = d; i < mat.n; ++i)
            {
                var r = mat.Rows[i];

                if (r.Index.Length > 0)
                {
                    var index = new List<int>();
                    var value = new List<double>();
                    var row = r.Elements;

                    for (int j = d; j < mat.m; j++)
                    {
                        if (row.TryGetValue(j, out double val))
                        {
                            index.Add(j);
                            value.Add(val);
                        }
                    }

                    res.Rows[i] = new SparseVector(value.ToArray(), index.ToArray(), mat.m);
                }
                else
                    res.Rows[i] = new SparseVector(new double[0], new int[0], mat.m);   
            }

            for (int i = 0; i < d; ++i)
                res.Rows[i] = new SparseVector(new double[] { 1.0 }, new int[] { i }, mat.m);
            
            return res;
        }

        private SparseMatrix ComputeHouseholderFactor(double[] v)
        {
            int n = v.Length;
            var mat = new SparseMatrix(n, n);

            for (int i = 0; i < n; i++)
            {
                var index = new List<int>();
                var value = new List<double>();

                for (int j = 0; j < n; j++)
                {
                    double val = -2.0 * v[i] * v[j];

                    if (val != 0.0 || i == j)
                    {
                        index.Add(j);
                        val += (i != j) ? 0.0 : 1.0;
                        value.Add(val);
                    }
                }

                mat.Rows[i] = new SparseVector(value.ToArray(), index.ToArray(), n);
            }
                        
            return mat;
        }

        private double[] ForwardSubstitution(SparseMatrix matrix, double[] vec)
        {
            var res = new double[vec.Length];

            for (int i = 0; i < matrix.n; i++)
            {
                var alpha = vec[i];
                for (int j = 0; j < i; j++)
                {
                    if (matrix.Rows[i].Elements.TryGetValue(j, out double value))
                        alpha -= value * res[j];
                }

                if (matrix.Rows[i].Elements.TryGetValue(i, out double value1))
                    res[i] = alpha / value1;
            }

            return res;
        }

        
        
        #endregion
    }
}
