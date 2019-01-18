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

using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.SolutionIntegration;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class NonLinearGaussSeidel : ISolver
    {
        #region Private Fields

        ProjectedGaussSeidel gaussSeidelSolver;
        LinearProblemBuilderEngine lcpEngine;
        IntegrateVelocity velocityIntegration;

        readonly SolverParameters solverParam;
                
        #endregion

        #region Constructor

        public NonLinearGaussSeidel(
            SolverParameters solverParameters,
            LinearProblemBuilderEngine lcpEngine,
            IntegrateVelocity velocityIntegration)
        {
            solverParam = solverParameters;
            this.lcpEngine = lcpEngine;
            this.velocityIntegration = velocityIntegration;

            var gaussSeidelSolverParam = new SolverParameters(
                                                          5,
                                                          solverParam.ErrorTolerance,
                                                          solverParam.SOR,
                                                          solverParam.MaxThreadNumber);

            gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public double[] Solve(
            LinearProblemProperties input,
            JacobianConstraint[] constraints,
            double[] x)
        {
            double[] oldX = new double[x.Length];
            double[] result = new double[x.Length];
            Array.Copy(x, result, x.Length);
            double actualSolverError = 0.0;

            result = gaussSeidelSolver.Solve(input, constraints, oldX);
            
            for (int i = 0; i < solverParam.MaxIterations; i++)
            {
                velocityIntegration.UpdateVelocity(constraints, result);

                //input = lcpEngine.UpdateVelocity(constraints, input);

                result = gaussSeidelSolver.Solve(input, constraints, new double[x.Length]);

                actualSolverError = SolverHelper.ComputeSolverError(result, oldX);

                if (actualSolverError < solverParam.ErrorTolerance)
                    break;

                Array.Copy(result, oldX, x.Length);
            }

            return result;
        }

        public SolverParameters GetSolverParameters()
        {
            return solverParam;
        }

        #endregion

        //      For n Newton iterations
        //        Update J, M and C using p
        //        For m Gauss-Seidel iterations
        //            For k constraints
        //                Solve lambda = -C / (J* M * J^ T)
        //                Update dp --- dp = M^-1 * J^T * lambda
        //                Update p  --- p+=dp
    }
}
