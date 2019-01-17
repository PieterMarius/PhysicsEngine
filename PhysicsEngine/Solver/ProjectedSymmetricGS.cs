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
using SharpEngineMathUtility.Solver;
using System;
using static SharpEngineMathUtility.SparseMatrix;
using static SharpEngineMathUtility.MathUtils;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.Helper;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class ProjectedSymmetricGS : ISolver
    {
        #region Fields

        public readonly SolverParameters SolverParameters;
        private readonly ProjectedGaussSeidel gsSolver;
        private readonly LinearProblemBuilderEngine lcpEngine;

        #endregion

        #region Constructor

        public ProjectedSymmetricGS(
            SolverParameters solverParameters,
            LinearProblemBuilderEngine lcpEngine)
        {
            SolverParameters = solverParameters;
           
            var gaussSeidelSolverParam = new SolverParameters(
                                                          1,
                                                          SolverParameters.ErrorTolerance,
                                                          1.0,
                                                          SolverParameters.MaxThreadNumber);

            gsSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
            this.lcpEngine = lcpEngine; 
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
        }

        public double[] Solve(
            LinearProblemProperties linearProblemProperties, 
            JacobianConstraint[] constraints, 
            double[] x)
        {
            JacobianConstraint[] constraintsRev = new JacobianConstraint[constraints.Length];
            Array.Copy(constraints, constraintsRev, constraints.Length);
            Array.Reverse(constraintsRev);
            var symLCP = lcpEngine.BuildLCP(constraintsRev, 0.015);

            double[] oldX = new double[x.Length];
            double[] result = new double[x.Length];
            Array.Copy(x, result, x.Length);
            double actualSolverError = 0.0;

            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                
            }

            

            return null;
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

        private double[] GetDirection(
            SparseMatrix A,
            double[] b,
            double[] x)
        {
            //return Minus(Multiply(A, x, SolverParameters.MaxThreadNumber), b);
            return Minus(b, Multiply(A, x, SolverParameters.MaxThreadNumber));
        }

        private void Clamp(
            LinearProblemProperties linearProblemProperties,
            ref double[] x)
        {
            for (int i = 0; i < linearProblemProperties.Count; i++)
            {

                ClampSolution.Clamp(linearProblemProperties, x[i], ref x, i);
                //if (x[i] < 0.0)
                //    x[i] = 0.0;
            }
        }

        #endregion
    }
}
