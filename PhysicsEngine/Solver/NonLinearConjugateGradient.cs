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

namespace SharpPhysicsEngine.LCPSolver
{
	internal sealed class NonLinearConjugateGradient : ISolver
    {
		#region Private Fields

		ProjectedGaussSeidel gaussSeidelSolver;

		public readonly SolverParameters solverParam;

		double deltaErrorCheck = -1.0;

		#endregion

        #region Constructor

        public NonLinearConjugateGradient(
            SolverParameters solverParameters)
        {
            solverParam = solverParameters;

			var gaussSeidelSolverParam = new SolverParameters (
				                                          1,
				                                          solverParam.ErrorTolerance,
				                                          1.0,
				                                          solverParam.MaxThreadNumber);
			
			gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public double[] Solve(
            LinearProblemProperties input,
            double[] x)
        {
            double[] Xk = gaussSeidelSolver.Solve(input, x);
            double[] delta = CalculateDelta(Xk, x);
            double[] searchDirection = NegateArray(delta);

            double[] Xk1 = new double[input.Count];

            for (int i = 0; i < solverParam.MaxIteration; i++)
            {
                Xk1 = gaussSeidelSolver.Solve(input, Xk);
                                
                double[] deltaK = CalculateDelta(Xk1, Xk);

                deltaErrorCheck = ArraySquareModule(delta);
                
                double betaK = 1.1;
				if (Math.Abs(deltaErrorCheck) > 1E-40)
                    betaK = ArraySquareModule(deltaK) / deltaErrorCheck;

				if (betaK > 1.0)
					searchDirection = new double[searchDirection.Length];
                else
                {
					Xk1 = CalculateDirection (
						input,
						Xk1, 
						deltaK, 
						ref searchDirection, 
						betaK);
                }

                Array.Copy(Xk1, Xk, Xk1.Length);
                Array.Copy(deltaK, delta, deltaK.Length);
            }
            return Xk1;
        }

		public double GetDifferentialMSE()
		{
			return deltaErrorCheck;
		}

		public SolverParameters GetSolverParameters()
		{
			return solverParam;
		}

        #endregion

        #region Private Methods

        private double[] CalculateDelta(
            double[] a,
            double[] b)
        {
			if (a.Length < 0 ||
				b.Length < 0 ||
				b.Length != a.Length)
			{
				throw new Exception("Different array size.");
			}

            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                result[i] = -(a[i] - b[i]);
            
            return result;
        }

        private double[] NegateArray(
            double[] a)
        {
            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
            {
                result[i] = -a[i];
            }

            return result;
        }

        private double ArraySquareModule(
            double[] a)
        {
            double mod = 0.0;
            double buf;
            for (int i = 0; i < a.Length; i++)
            {
                buf = a[i];
                mod += buf * buf; 
            }
            return mod;
        }

        private double[] CalculateDirection(
			LinearProblemProperties input,
			double[] Xk1,
            double[] deltaK,
            ref double[] searchDirection,
            double betak)
        {
            double[] result = new double[Xk1.Length];

            for (int i = 0; i < Xk1.Length; i++)
            {
                double bDirection = betak * searchDirection[i];

                result[i] = Xk1[i] + bDirection;

                searchDirection[i] = bDirection - deltaK[i];

                result[i] = ClampSolution.Clamp(input, result[i], result, i);
            }

            return result;
        }

		#endregion
	}
}
