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
using static SharpEngineMathUtility.MathUtils;
using static SharpEngineMathUtility.SparseMatrix;
using static SharpEngineMathUtility.GeometryUtils;

namespace SharpEngineMathUtility.Solver
{
    public sealed class Lemke
    {
        #region Fields

        const double threshold = 1E-5;

        private HouseholderQR HouseholderQRSolver;
        private GMRES GMRESSolver;
        private LUSolver luSolver;
        private ConjugateGradient CG;

        private readonly SolverType Type;

        #endregion

        #region Constructor

        public Lemke(SolverType type)
        {
            Type = type;
            InitSolverType(type);
        }

        #endregion

        #region Public Methods

        public double[] Solve(
            SparseMatrix M,
            double[] q,
            int maxIter,
            double[] z0 = null)
        {
            int n = q.Length;
            double zero_tol = 1E-5;
            double piv_tol = 1E-12;
            double err = 0.0;
            double[] _z = null; 

            if (CheckPositiveValues(q))
                return new double[n];

            double[] z = new double[2 * n];
            double ratio = 0.0;
            int leaving = 0;

            double[] x = new double[q.Length];
            Array.Copy(q, x, q.Length);
            List<int> bas = new List<int>();
            List<int> nonbas = new List<int>();

            int t = 2 * n;
            int entering = t;

            if (z0 != null)
            {
                for (int i = 0; i < n; ++i)
                {
                    if (z0[i] <= 0.0)
                        nonbas.Add(i);
                    else
                        bas.Add(i);
                }
            }
            else
            {
                for (int i = 0; i < n; ++i)
                    nonbas.Add(i);
            }

            var B = GetSparseIdentityMatrix(n, n, -1.0);

            if (bas.Count > 0)
            {
                var BCopy = GetCopy(B);

                for (int i = 0; i < bas.Count; i++)
                    SetColumn(ref B, GetColumn(M, bas[i]), i);

                for (int i = 0; i < nonbas.Count; i++)
                    SetColumn(ref B, GetColumn(BCopy, nonbas[i]), bas.Count + i);

                x = Multiply(-1.0, HouseholderQRSolver.Solve(B, q));
            }

            if (CheckPositiveValues(x))
            {
                var __z = new double[2 * n];
                for (int i = 0; i < bas.Count; ++i)
                    __z[bas[i]] = x[i];
                
                Array.Copy(__z, _z, n);

                return _z;
            }

            double[] minuxX = Multiply(-1.0, x);
            int lvindex = -1;
            double tval = Max(minuxX, ref lvindex);

            for (int i = 0; i < nonbas.Count; ++i)
                bas.Add(nonbas[i] + n);

            leaving = bas[lvindex];
            bas[lvindex] = t;

            var U = new double[n];
            for (int i = 0; i < n; ++i)
            {
                if (x[i] < 0.0)
                    U[i] = 1.0;
            }

            var Be = Multiply(-1.0, Multiply(B, U));
            x = Add(x, Multiply(tval, U));
            x[lvindex] = tval;
            SetColumn(ref B, Be, lvindex);
                       
            int iter = 0;

            for (iter = 0; iter < maxIter; ++iter)
            {
                if (leaving == t)
                {
                    break;
                }
                else if (leaving < n)
                {
                    entering = n + leaving;
                                       
                    Be = new double[n];
                    Be[leaving] = -1.0;
                }
                else
                {
                   
                    entering = leaving - n;
                               
                    Be = GetColumn(M, entering);
                }

                var d = ExecuteSolver(B, Be);

                List<int> j = new List<int>();
                for (int i = 0; i < n; ++i)
                {
                    if (d[i] > piv_tol)
                    {
                        j.Add(i);
                    }
                }

                if (!j.Any())
                    break;
                
                int jSize = j.Count;
                double[] minRatio = new double[jSize];

                for (int i = 0; i < jSize; ++i)
                {
                    int index = j[i];
                    minRatio[i] = (x[index] + zero_tol) / d[index];
                }

                double theta = Min(minRatio);

                List<int> tmpJ = new List<int>();
                List<double> tmpd = new List<double>();

                for (int i = 0; i < jSize; ++i)
                {
                    int index = j[i];
                    if (x[index] / d[index] <= theta)
                    {
                        tmpJ.Add(index);
                        tmpd.Add(d[index]);
                    }
                }

                j = tmpJ;
                jSize = j.Count;
                if (jSize == 0)
                    break;
                
                lvindex = -1;
                for (int i = 0; i < jSize; i++)
                {
                    if (bas[j[i]] == t)
                        lvindex = i;
                }

                if (lvindex != -1)
                    lvindex = j[lvindex];
                else
                {
                    theta = tmpd[0];
                    lvindex = 0;
                    
                    for (int i = 0; i < jSize; ++i)
                    {
                        var tdiff = tmpd[i] - theta;
                        if (tdiff > piv_tol)
                        {
                            theta = tmpd[i];
                            lvindex = i;
                            
                        }
                    }

                    var theta1 = Max(d);
                    var possibleLeave = new List<int>();
                    for (int i = 0; i < jSize; i++)
                    {
                        int index = j[i];
                        if (Math.Abs(d[index] - theta1) < zero_tol)
                        {
                            possibleLeave.Add(index);
                        }
                    }
                    if (possibleLeave.Count > 1)
                    {
                        var count = GetRandom(0, possibleLeave.Count);
                        lvindex = possibleLeave[count];
                    }
                    else
                    {

                        lvindex = j[lvindex];
                    }
                }

                leaving = bas[lvindex];
                ratio = x[lvindex] / d[lvindex];

                x = Minus(x, Multiply(ratio, d));
                x[lvindex] = ratio;
                SetColumn(ref B, Be, lvindex);

                bas[lvindex] = entering;
            }

            //End Iteration region

            if (iter >= maxIter && leaving != t)
                throw new Exception("Lemke solver failed to find solution.");

            if (err == 0.0)
            {
                for (int i = 0; i < bas.Count; i++)
                {
                    if (bas[i] < z.Length)
                        z[bas[i]] = x[i];
                }
                _z = new double[n];
                Array.Copy(z, _z, n);

                var validation = Validate(M, _z, q);

                if (!validation)
                    throw new Exception("Lemke solver failed to find solution.");
            }

            return _z;
        }

        #endregion

        #region Private Methods

        private void InitSolverType(SolverType type)
        {
            switch(type)
            {
                case SolverType.GMRES:
                    GMRESSolver = new GMRES();
                    break;

                case SolverType.LUSolver:
                    luSolver = new LUSolver();
                    GMRESSolver = new GMRES();
                    break;

                case SolverType.ConjugateGradient:
                    CG = new ConjugateGradient();
                    break;

                case SolverType.HouseHolderQR:
                default:
                    HouseholderQRSolver = new HouseholderQR();
                    break;
            }
        }

        private double[] ExecuteSolver(
            SparseMatrix A,
            double[] b)
        {
            switch(Type)
            {
                case SolverType.GMRES:
                    int maxIter = 2000;
                    return GMRESSolver.Solve(A, b, new double[b.Length], maxIter, b.Length);

                case SolverType.LUSolver:

                    var solution = luSolver.Solve(A, b, out bool valid);

                    if (!valid)
                    {
                        int maxIter1 = Math.Min(1000, 4000);
                        solution = GMRESSolver.Solve(A, b, solution, maxIter1, b.Length);
                    }

                    return solution;

                case SolverType.ConjugateGradient:
                    SparseMatrix At = Transpose(A);
                    
                    SparseMatrix AA = Multiply(At, A);
                    double[] bt = Multiply(At, b);
                    
                    return CG.Solve(AA, bt, new double[bt.Length], 1000);

                case SolverType.HouseHolderQR:
                default:
                    return HouseholderQRSolver.Solve(A, b);
            }
        }

        private bool CheckPositiveValues(double[] q)
        {
            var res = true;
            for (int i = 0; i < q.Length; i++)
            {
                if (q[i] < 0.0)
                {
                    res = false;
                    break;
                }
            }

            return res;
        }

        private bool Validate(
            SparseMatrix M,
            double[] z,
            double[] q)
        {
            var w = Add(q, Multiply(M, z));
            for (int i = 0; i < M.n; i++)
            {
                if (w[i] < -threshold || z[i] < -threshold)
                    return false;

                if (Math.Abs(w[i] * z[i]) > threshold)
                    return false;
            }
            return true;
        }
                
        #endregion
    }
}
