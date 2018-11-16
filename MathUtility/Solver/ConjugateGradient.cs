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
    public sealed class ConjugateGradient
    {

        #region Public Methods

        public double[] Solve(
            SparseMatrix A,
            double[] b,
            double[] x,
            int nIter,
            double tol = 1E-12)
        {
            double[] xi = x;
            double[] rNew = Minus(b, Multiply(A, x));
            double[] p = rNew;
            double r2Old = Dot(rNew, rNew);

            double alpha = 1.0;
            double beta = 1.0;

            for (int i = 0; i < nIter; i++)
            {
                alpha = GetAlpha(A, p, r2Old);

                xi = Add(xi, Multiply(alpha, p));

                rNew = Minus(rNew, Multiply(alpha, Multiply(A, p)));

                double r2New = Dot(rNew, rNew);

                if (r2New < tol)
                    return xi;

                beta = GetBeta(r2New, r2Old);

                p = Add(rNew, Multiply(beta, p));
                r2Old = r2New;
            }

            return xi;
        }

        #endregion

        #region Private Methods

        private double GetAlpha(
            SparseMatrix A,
            double[] p,
            double num)
        {
            var denom = Dot(p, Multiply(A, p));

            if (denom == 0.0)
                return 1.0;

            return num / denom;
        }
        public double GetBeta(
            double num,
            double denom)
        {
            if (denom == 0.0)
                return 1.0;

            return num / denom;
        }


        #endregion
    }
}
