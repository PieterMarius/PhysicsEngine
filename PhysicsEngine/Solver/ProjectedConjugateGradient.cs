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

using SharpEngineMathUtility;
using static SharpEngineMathUtility.SparseMatrix;
using static SharpEngineMathUtility.MathUtils;
using System.Linq;
using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;
using static SharpPhysicsEngine.LCPSolver.RedBlackProjectedGaussSeidel;
using System;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class ProjectedConjugateGradient : ISolver
    {
        #region Fields

        RedBlackProjectedGaussSeidel gaussSeidelSolver;
        public readonly SolverParameters SolverParameters;

        #endregion

        #region Constructor

        public ProjectedConjugateGradient(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;

            var gaussSeidelSolverParam = new SolverParameters(
                                                          1,
                                                          SolverParameters.ErrorTolerance,
                                                          1.0,
                                                          SolverParameters.MaxThreadNumber);

            gaussSeidelSolver = new RedBlackProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
        }

        public double[] Solve(
            LinearProblemProperties linearProblemProperties,
            double[] startValues)
        {
            if (startValues == null)
                startValues = new double[linearProblemProperties.Count];

            double[] x = startValues;

            SparseMatrix A = linearProblemProperties.GetOriginalSparseMatrix();
            double[] r = GetDirection(A, linearProblemProperties.B, x);

            if (Dot(r, r) < 1E-50)
                return startValues;

            double[] p = r;

            var checkBoundConstraints = linearProblemProperties.ConstraintType.Any(k =>
                k == ConstraintType.Friction || k == ConstraintType.JointLimit || k == ConstraintType.JointMotor);
            var checkUnboundConstraints = linearProblemProperties.ConstraintType.Any(k =>
                k == ConstraintType.Joint || k == ConstraintType.SoftJoint);

            Dictionary<RedBlackEnum, List<int>> redBlackDictionary = null;

            //if (checkBoundConstraints)
            redBlackDictionary = gaussSeidelSolver.GetRedBlackDictionary(linearProblemProperties);
            gaussSeidelSolver.SolverParameters.SetSolverMaxIteration(5);
            x = gaussSeidelSolver.SolveExecute(linearProblemProperties, redBlackDictionary, x);
            gaussSeidelSolver.SolverParameters.SetSolverMaxIteration(1);

            double[] xOld = new double[x.Length];
            
            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                double[] Ap = Multiply(A, p, SolverParameters.MaxThreadNumber);

                double alphaCG = GetAlphaCG(Ap, r, p);

                //x = UpdateSolution(linearProblemProperties, x, p, alphaCG);
                x = gaussSeidelSolver.SolveExecute(linearProblemProperties, redBlackDictionary, x);
                
                double actualSolverError = SolverHelper.ComputeSolverError(x, xOld);
                if (actualSolverError < SolverParameters.ErrorTolerance)
                    return x;

                Array.Copy(x, xOld, x.Length);

                r = GetDirection(A, linearProblemProperties.B, x);

                //double[] phiY = GetPhi(linearProblemProperties, x, r);
                double[] phiY = r;
                double[] partialValue = Ap;
                double denom = Dot(p, partialValue);
                double beta = 0.0;
                if (denom != 0.0)
                    beta = Dot(phiY, partialValue) / denom;

                p = Minus(phiY, ParallelMultiply(beta, p, SolverParameters.MaxThreadNumber));
            }
            //var erra = CheckErrorTest(x, A, linearProblemProperties);
            //for (int j = 0; j < x.Length; j++)
            //{
            //    x[j] = ClampSolution.Clamp(linearProblemProperties, x[j], x, j);
            //    //x = gaussSeidelSolver.SolveExecute(linearProblemProperties, redBlackDictionary, x);  
            //}

            var errp = CheckErrorTest(x, A, linearProblemProperties);
            if(errp > 1E-1)
            {
                var tttt = false;
            }

            return x;
        }

        #endregion

        #region Private Methods

        private double CheckErrorTest(double[] x, SparseMatrix A, LinearProblemProperties input)
        {
            double[] xValue = x;
            double[] dir = GetDirection(A, input.B, xValue);

            double error = 0.0;

            for (int i = 0; i < dir.Length; i++)
            {
                if (input.ConstraintType[i] == ConstraintType.Collision)
                {
                    //error += dir[i] * dir[i];
                }
                else if (input.ConstraintType[i] == ConstraintType.Joint)
                    error += dir[i] * dir[i];

                else if (input.ConstraintType[i] == ConstraintType.SoftJoint)
                    error += dir[i] * dir[i];
                //else if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.Friction)
                //{
                //    double? min = 0.0;
                //    double? max = 0.0;

                //    ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                //    if (min.HasValue && xValue[i] + 1E-9 < min)
                //        error += 0.0;
                //    else if (max.HasValue && xValue[i] - 1E-9 > max)
                //        error += 0.0;
                //}
            }

            return error;
        }

        /// <summary>
        /// Ax - b
        /// </summary>
        /// <param name="A"></param>
        /// <param name="b"></param>
        /// <param name="x"></param>
        /// <returns></returns>
        private double[] GetDirection(
            SparseMatrix A,
            double[] b,
            double[] x)
        {
            return Minus(Multiply(A, x, SolverParameters.MaxThreadNumber), b);
        }

        /// <summary>
        /// (g^t * p) / (p^t *A * p)
        /// </summary>
        /// <param name="g"></param>
        /// <param name="p"></param>
        /// <param name="A"></param>
        /// <returns></returns>
        private double GetAlphaCG(
            double[] Ap,
            double[] g,
            double[] p)
        {
            double den = Dot(Ap, p);
            double alphaCG = 0.0;

            if (den != 0)
                return Dot(g, p) / den;

            return alphaCG;
        }

        /// <summary>
        /// x - alpha * p
        /// </summary>
        /// <param name="x"></param>
        /// <param name="p"></param>
        /// <param name="alpha"></param>
        /// <returns></returns>
        private double[] UpdateSolution(
            LinearProblemProperties lp,
            double[] x,
            double[] p,
            double alpha)
        {
            var pr = ParallelMultiply(alpha, p, SolverParameters.MaxThreadNumber);
            var res = x;

            for (int i = 0; i < lp.Count; i++)
            {
                if (lp.ConstraintType[i] == ConstraintType.Joint &&
                    lp.ConstraintType[i] == ConstraintType.SoftJoint)
                    res[i] -= pr[i];
            }

            return res;
        }

        private double[] GetPhi(
            LinearProblemProperties input,
            double[] x,
            double[] g)
        {
            double[] result = new double[input.Count];

            for (int i = 0; i < input.Count; i++)
            {
                if (!ClampSolution.GetIfClamped(input, x, i))
                    result[i] = g[i];
                else
                    result[i] = 0.0;
            }

            return result;
        }

        #endregion
    }
}
