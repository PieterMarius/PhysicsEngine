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
using System.Collections.Concurrent;
using System.Linq;
using System.Threading.Tasks;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class ProjectedGaussSeidel : ISolver
    {
		#region Fields

		public readonly SolverParameters SolverParameters;

		#endregion

        #region Constructor

        public ProjectedGaussSeidel(
			SolverParameters solverParameters)
        {
			SolverParameters = solverParameters;
        }

        #endregion

        #region Public Methods

        public double[] Solve(
            LinearProblemProperties input,
            JacobianConstraint[] constraints,
            double[] x)
        {
            var solverOrder = Enumerable.Range(0, x.Length).ToArray();
            return Execute(input, x, solverOrder);
        }

        public double[] Solve(
            LinearProblemProperties input,
            double[] x,
            int[] solverOrder)
        {
            return Execute(input, x, solverOrder);
        }

        public void SetSuccessiveOverRelaxation(double SOR)
        {
			SolverParameters.SetSOR(SOR);
        }

		public SolverParameters GetSolverParameters()
		{
			return SolverParameters;
		}

        #endregion

        #region Private Methods

        private double[] Execute(
            LinearProblemProperties input,
            double[] x,
            int[] solverOrder)
        {
            double[] oldX = new double[x.Length];
            double[] result = new double[x.Length];
            Array.Copy(x, result, x.Length);
            double actualSolverError = 0.0;
                       
            for (int k = 0; k < SolverParameters.MaxIterations; k++)
            {
                var sum = ElaborateLowerTriangularMatrix(input, result);

                actualSolverError = ElaborateUpperTriangularMatrix(
                    input, 
                    sum, 
                    solverOrder, 
                    ref result, 
                    ref oldX);

                if (actualSolverError < SolverParameters.ErrorTolerance)
                    break;
            }

            return result;
        }

        private double ElaborateUpperTriangularMatrix(
            LinearProblemProperties input,
            double[] sum,
            int[] solverOrder,
            ref double[] x,
            ref double[] oldX)
        {
            double error = 0.0;

            for (int i = 0; i < input.Count; i++)
            {
                int index = solverOrder[i];

                double sumBuffer = sum[index];

                SparseVector m = input.M.Rows[index];
                ShapeDefinition.ConstraintType cType = input.ConstraintType[index];

                double xValue = x[index];

                //Avoid first row elaboration
                if (index != 0 && m.Index.Length > 0)
                {
                    var bufValue = m.Value;
                    var bufIndex = m.Index;

                    for (int j = 0; j < bufIndex.Length; j++)
                    {
                        int idx = bufIndex[j];
                        if (idx < index)
                            sumBuffer += bufValue[j] * x[idx];
                    }
                }

                sumBuffer = (input.B[index] - sumBuffer) * input.InvD[index];

                double sor = SolverParameters.SOR;

                // Constraints with limit diverge with sor > 1.0
                if (sor > 1.0 &&
                    (cType == ShapeDefinition.ConstraintType.Friction ||
                    cType == ShapeDefinition.ConstraintType.JointMotor ||
                    cType == ShapeDefinition.ConstraintType.JointLimit))
                    sor = 1.0;
                
                xValue += (sumBuffer - xValue) * sor;

                x[index] = ClampSolution.Clamp(input, xValue, ref x, index);

                //Compute error
                double diff = x[index] - oldX[index];
                error += diff * diff;
                oldX[index] = x[index];
            }

            return error;
        }

        private double[] ElaborateLowerTriangularMatrix(
            LinearProblemProperties input,
            double[] x)
        {
            double[] sum = new double[input.Count];

            for (int i = 0; i < input.Count; i++)
                sum[i] = Kernel(input, x, i);
            
            return sum;
        }

        static double Kernel(
            LinearProblemProperties input,
            double[] x,
            int i)
        {
			double sumBuffer = 0.0;

			//Avoid last row elaboration
			if (i + 1 != input.Count &&
                input.M.Rows[i].Index.Length > 0)
            {
                var bufValue = input.M.Rows[i].Value;
                var bufIndex = input.M.Rows[i].Index;

                for (int j = 0; j < bufIndex.Length; j++)
                {
                    int idx = bufIndex[j];
                    if (idx > i)
                        sumBuffer += bufValue[j] * x[idx];
                }
			}
            return sumBuffer;
        }

        #endregion
    }
}
