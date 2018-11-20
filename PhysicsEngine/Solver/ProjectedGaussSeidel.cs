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
            double[] x)
        {
            double[] oldX = new double[x.Length];
            double[] result = new double[x.Length];
            Array.Copy(x, result, x.Length);
            double actualSolverError = 0.0;
            
            var rangePartitioner = Partitioner.Create(0, input.Count, Convert.ToInt32(input.Count / SolverParameters.MaxThreadNumber) + 1);
            for (int k = 0; k < SolverParameters.MaxIteration; k++)
            {
                ElaborateUpperTriangularMatrix(input, rangePartitioner, ref result);

                actualSolverError = SolverHelper.ComputeSolverError(result, oldX);

                if (actualSolverError < SolverParameters.ErrorTolerance)
                    return result;

                Array.Copy(result, oldX, result.Length);
            }
            
            return result;
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

        private void ElaborateUpperTriangularMatrix(
            LinearProblemProperties input,
            OrderablePartitioner<Tuple<int, int>> rangePartitioner,
            ref double[] x)
        {
            double[] sum = ElaborateLowerTriangularMatrix(input, x, rangePartitioner);

            for (int i = 0; i < input.Count; i++)
            {
                double sumBuffer = sum[i];

                SparseVector m = input.M.Rows[i];

                double xValue = x[i];

                //Avoid first row elaboration
                if (i != 0)
                {
                    var bufValue = m.Value;
                    var bufIndex = m.Index;

                    for (int j = 0; j < bufIndex.Length; j++)
                    {
                        int idx = bufIndex[j];
                        if (idx < i)
                            sumBuffer += bufValue[j] * x[idx];
                    }
                }

                sumBuffer = (input.B[i] - sumBuffer) * input.InvD[i];

                xValue += (sumBuffer - xValue) * SolverParameters.SOR;

                x[i] = ClampSolution.Clamp(input, xValue, x, i);
            }
        }

        private double[] ElaborateLowerTriangularMatrix(
            LinearProblemProperties input,
            double[] x,
            OrderablePartitioner<Tuple<int, int>> rangePartitioner)
        {
            double[] sum = new double[input.Count];
                        
            Parallel.ForEach(
                    rangePartitioner,
                    new ParallelOptions { MaxDegreeOfParallelism = SolverParameters.MaxThreadNumber },
                    (range, loopState) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                        sum[i] = Kernel(input, x, i);
                });

            return sum;
        }

        static double Kernel(
            LinearProblemProperties input,
            double[] x,
            int i)
        {
			double sumBuffer = 0.0;

			//Avoid last row elaboration
			if (i + 1 != input.Count)
            {
                if (input.M.Rows[i].Index.Length > 0)
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
			}
            return sumBuffer;
        }

        #endregion
    }
}
