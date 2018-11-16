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


namespace SharpEngineMathUtility.Solver
{
    public sealed class LUSolver
    {

        #region Constructor

        public LUSolver()
        { }

        #endregion

        #region Public Methods

        public double[] Solve(SparseMatrix A, double[] b, out bool valid)
        {
            double[,] lu = new double[A.n, A.n];
            double sum = 0.0;
            valid = true;

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
                else
                    valid = false;
                
                x[i] = v * (y[i] - sum);
            }

            return x;

        }

        #endregion

    }
}
