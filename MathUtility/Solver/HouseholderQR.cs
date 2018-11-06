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
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static SharpEngineMathUtility.MathUtils;
using static SharpEngineMathUtility.SparseMatrix;

namespace SharpEngineMathUtility.Solver
{
    public sealed class HouseholderQR
    {

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

            for (int k = 0; k < n && k < m - 1; k++)
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
                    e = Multiply(1.0 / Module(e), e);
                
                qv.Add(ComputeHouseholderFactor(e));

                z = Multiply(
                    qv[qv.Count - 1],
                    z1);
            }

            var Q = qv[0];

            for (int i = 1; i < n && i < m - 1; i++)
            {
                z1 = Multiply(qv[i], Q);
                Q = z1;
            }
            
            var R = Multiply(Q, mat);

            Q = Transpose(Q);

            return new QRDecomposition() { Q = Q, R = R };
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

                for (int j = i + 1; j < rows; j++)
                {
                    if (matrix.Rows[i].Elements.TryGetValue(j, out double value))
                        res[i] -= value * res[j];
                }
                if (matrix.Rows[i].Elements.TryGetValue(i, out double value1))
                    res[i] = res[i] / value1;
            }

            return res;

        }

        private SparseMatrix ComputeMinor(SparseMatrix mat, int d)
        {
            var res = new SparseMatrix(mat.n, mat.m);

            for (int i = d; i < mat.n; i++)
            {
                var index = new List<int>();
                var value = new List<double>();
                for (int j = d; j < mat.m; j++)
                {      
                    if (mat.Rows[i].Elements.TryGetValue(j, out double val))
                    {
                        index.Add(j);
                        value.Add(val);
                    }
                }

                res.Rows[i] = new SparseVector(value.ToArray(), index.ToArray(), mat.m);   
            }

            for (int i = 0; i < d; i++)
            {
                if (res.Rows[i].Length == 0)
                    res.Rows[i] = new SparseVector(new double[] { 1.0 }, new int[] { i }, mat.m);
            }

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
                    index.Add(j);
                    double val = -2.0 * v[i] * v[j];
                    if (i == j)
                        val += 1.0;
                    value.Add(val);
                }

                mat.Rows[i] = new SparseVector(value.ToArray(), index.ToArray(), n);
            }
                        
            return mat;
        }

        #endregion
    }
}
