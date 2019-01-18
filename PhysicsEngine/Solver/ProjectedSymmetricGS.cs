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
                                                          2,
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
            UpdateFrictionConstraints(symLCP);

            double[] oldX = new double[x.Length];
            double[] result = new double[x.Length];
            double[] resultRev = new double[x.Length];
            Array.Copy(x, result, x.Length);
            double actualSolverError = 0.0;

            for (int i = 0; i < SolverParameters.MaxIterations; i++)
            {
                result = gsSolver.Solve(linearProblemProperties, constraints, result);
                
                Array.Reverse(result);

                result = gsSolver.Solve(symLCP, constraints, result);

                Array.Reverse(result);

                actualSolverError = SolverHelper.ComputeSolverError(result, oldX);

                if (actualSolverError < SolverParameters.ErrorTolerance)
                    break;

                Array.Copy(result, oldX, x.Length);
            }
            
            return result;
        }

        #endregion

        #region Private Methods

        private void UpdateFrictionConstraints(LinearProblemProperties lcp)
        {
            var frictionRes = new int?[lcp.Count];

            for (int i = 0; i < lcp.Constraints.Length; i++)
            {
                if (lcp.ConstraintType[i] == ConstraintType.Collision)
                {
                    for (int j = 0; j < lcp.FrictionDirections; j++)
                    {
                        frictionRes[i - j - 1] = -i;
                        
                    }
                    
                }
            }
            lcp.SetConstraints(frictionRes);

        }

        

        #endregion
    }
}
