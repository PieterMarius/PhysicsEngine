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
using static SharpEngineMathUtility.MathUtils;
using static SharpEngineMathUtility.SparseMatrix;

namespace SharpEngineMathUtility.Solver
{
    public sealed class MINRES
    {
        #region Fields
                        
        #endregion

        #region Constructor

        public MINRES()
        { }

        #endregion

        #region Public Methods
                
        public double[] Solve(
            SparseMatrix A,
            double[] b,
            double[] x,
            int nIter,
            double tol = 1E-10)
        {
            double[] v0 = new double[x.Length];
            double[] v1 = Minus(b, Multiply(A, x));

            if (Dot(v1, v1) < tol)
                return x;

            double beta1 = Math.Sqrt(Dot(v1, v1));
            double betaN = 0.0;
            double n = beta1;
            double c0 = 1.0;
            double c1 = 1.0;
            double s0 = 0.0;
            double s1 = 0.0;
            double[] w0 = new double[v1.Length];
            double[] w_1 = new double[v1.Length];
            double residual = double.NaN;
            
            for (int i = 0; i < nIter; i++)
            {
                //Calculate Lanczos Vectors
                double[] v = Multiply((1.0 / beta1), v1);
                double[] Av = Multiply(A, v);
                double alpha = Dot(v, Av);
                v1 = Minus(Minus(Av, Multiply(alpha, v)), Multiply(beta1, v0));
                betaN = Math.Sqrt(Dot(v1, v1));

                //Calculate QR factors
                double lambda = c1 * alpha - c0 * s1 * beta1;
                double p1 = Math.Sqrt(lambda * lambda + betaN * betaN);
                double p2 = s1 * alpha + c0 * c1 * beta1;
                double p3 = s0 * beta1;

                //Calculate New Givens Rotations
                c0 = c1;
                c1 = lambda / p1;

                s0 = s1;
                s1 = betaN / p1;

                //Update Solution
                double[] w = Multiply((1.0 / p1), (Minus(Minus(v, Multiply(p3, w_1)), Multiply(p2, w0))));

                x = Add(x, Multiply(c1, Multiply(n, w)));
                
                n = -s1 * n;

                residual = Math.Abs(n);

                if (residual < tol)
                    break;

                beta1 = betaN;
                v0 = v;
                w_1 = w0;
                w0 = w;
            }
                        
            return x;
        }

        #endregion

        #region Private Methods
                                
        #endregion
    }
}
