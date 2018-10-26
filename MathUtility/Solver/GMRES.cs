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
using static SharpEngineMathUtility.GeneralMathUtils;

namespace SharpEngineMathUtility.Solver
{
    public sealed class GMRES
    {
        #region Constructor

        public GMRES()
        {}

        #endregion

        #region Public Methods

        public double[] Solve(
            SparseElement[] A,
            double[] b,
            int restrt)
        {
            var x = new double[b.Length];

            return Solve(A, b, x, 1000, restrt);
        }

        public double[] Solve(
            SparseElement[] A,
            double[] b,
            double[] x,
            int maxIter,
            int restrt,
            double tol = 1E-10)
        {
            int i, j = 1, k;

            double bnrm2 = Normalize(b);
            if (bnrm2 == 0.0)
                bnrm2 = 1.0;

            double[] r = Minus(b, SparseElement.Multiply(A, x));
            double beta = Normalize(r);
            double error = beta / bnrm2;
            if (error < tol)
                return x;
                        
            int m = restrt;

            double[][] H = new double[m + 1][];
            for (int z = 0; z < m + 1; z++)
                H[z] = new double[m];
            
            double[][] v = new double[m + 1][];
            double[] cs = new double[m + 1];
            double[] sn = new double[m + 1];
            double[] s = new double[m + 1];

            while (j <= maxIter)
            {
                v[0] = Multiply(1.0 / beta, r);
                s[0] = beta;
                
                for (i = 0; i < m && j <= maxIter; i++, j++)
                {
                    double[] w = SparseElement.Multiply(A, v[i]);

                    for (k = 0; k <= i; k++)
                    {
                        H[k][i] = Dot(w, v[k]);
                        w = Minus(w, Multiply(H[k][i], v[k]));
                    }

                    H[i + 1][i] = Normalize(w);
                    v[i + 1] = Multiply(1.0 / H[i + 1][i], w);
                    
                    for (k = 0; k < i; k++)
                    {
                        var rtt = ApplyPlaneRotation(H[k][i], H[k + 1][i], cs[k], sn[k]);
                        H[k + 1][i] = rtt.y;
                        H[k][i] = rtt.x;
                    }

                    Vector2d rot = RotMat(H[i][i], H[i + 1][i]);
                    cs[i] = rot.x; sn[i] = rot.y;

                    var r1 = ApplyPlaneRotation(H[i][i], H[i + 1][i], cs[i], sn[i]);
                    H[i][i] = r1.x;
                    H[i + 1][i] = r1.y;

                    var r2 = ApplyPlaneRotation(s[i], s[i + 1], cs[i], sn[i]);
                    s[i] = r2.x;
                    s[i + 1] = r2.y;
                                        
                    error = Math.Abs(s[i + 1]) / bnrm2;
                    if (error <= tol)
                    {
                        x = Update(x, i, H, s, v);
                        maxIter = j;
                        
                        return x;
                    }
                }
                
                x = Update(x, i - 1, H, s, v);
                r = Minus(b, SparseElement.Multiply(A, x));
                beta = Normalize(r);
                error = beta / bnrm2;
                if (error <= tol)
                {
                    maxIter = j;
                    return x;
                }
                s = new double[m + 1];
            }

            return x;
        }

        #endregion

        #region Private Methods
                
        private Vector2d RotMat(double dx, double dy)
        {
            if (dy == 0.0)
            {
                return new Vector2d(1.0, 0.0);
            }
            else if(Math.Abs(dy) > Math.Abs(dx))
            {
                double temp = dx / dy;
                double sn = 1.0 / Math.Sqrt(1.0 + temp * temp);
                return new Vector2d(temp * sn, sn);
            }
            else
            {
                double temp = dy / dx;
                double cs = 1.0 / Math.Sqrt(1.0 + temp * temp);
                return new Vector2d(cs, cs * temp);
            }
        }

        private Vector2d ApplyPlaneRotation(
            double dx,
            double dy,
            double cs,
            double sn)
        {
            double temp = cs * dx + sn * dy;
            dy = -sn * dx + cs * dy;
            dx = temp;
            return new Vector2d(dx, dy);
        }

        private double[] Update(
            double[] x, 
            int k, 
            double[][] h, 
            double[] s,
            double[][] v)
        {
            double[] y = new double[s.Length];
            Array.Copy(s, y, s.Length);

            for (int i = k; i >= 0; i--)
            {
                y[i] /= h[i][i];
                for (int j = i - 1; j >= 0; j--)
                    y[j] -= h[j][i] * y[i];

            }

            for (int i = 0; i <= k ; i++)
            {
                x = Add(x, Multiply(y[i], v[i]));
            }

            return x;
        }

        #endregion
    }
}
