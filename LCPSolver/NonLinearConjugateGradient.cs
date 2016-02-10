using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LCPSolver
{
	public class NonLinearConjugateGradient : ISolver
    {
		private GaussSeidel gaussSeidelSolver;

		private readonly SolverParameters solverParameters;

        #region Constructor

        public NonLinearConjugateGradient(
            SolverParameters solverParameters)
        {
            this.solverParameters = solverParameters;

			SolverParameters gaussSeidelSolverParam = new SolverParameters (
				                                          1,
				                                          1E-20,
				                                          this.solverParameters.SOR,
														  this.solverParameters.MaxThreadNumber);
			
			this.gaussSeidelSolver = new GaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods


		//TODO da correggere
        public double[] Solve(LinearProblemProperties input)
        {
            double[] Xk = this.gaussSeidelSolver.Solve(input);
            double[] delta = this.calculateDelta(Xk, input.StartX);
            double[] searchDirection = this.negateArray(delta);

            input.SetStartValue(Xk);

            for (int i = 0; i < this.solverParameters.MaxIteration; i++)
            {
                double[] Xk1 = this.gaussSeidelSolver.Solve(input);
                                
                double[] deltaK = this.calculateDelta(Xk1, Xk);

                double deltaCheck = this.arraySquareModule(delta);

				//early exit
				if (deltaCheck < this.solverParameters.ErrorTolerance)
					return Xk1;

                double betaK = 0.0;
                if (deltaCheck != 0.0)
                    betaK = this.arraySquareModule(deltaK) / deltaCheck;

				if (betaK > 1.0)
					searchDirection = new double[searchDirection.Length];
                else
                {
					Xk = this.calculateDirection (
						input,
						Xk1, 
						deltaK, 
						ref searchDirection, 
						betaK);

                    input.SetStartValue(Xk);
                }
					
				for (int j = 0; j < deltaK.Length; j++) 
				{
					delta [j] = deltaK [j];
				}
            }
            return Xk;
        }


		public double[] GetError(
			LinearProblemProperties input,
			double[] X)
		{
			if (input.Count > 0) {
				double[] result = new double[input.Count];
				for (int i = 0; i < input.Count; i++) {
					result [i] = 0.0;
					for (int j = 0; j < input.Count; j++) {
						result [i] += input.M [i].Value[j] * X [input.M [i].Index[j]];
					}
					result [i] = (input.B [i] - result [i]);
					result [i] = result [i] * result [i];
				}

				return result;
			} 
			else 
			{
				throw new Exception ("Empty solver parameters");
			}
		}

		public double GetMediumSquareError (
			LinearProblemProperties input, 
			double[] X)
		{
			double[] errors = this.GetError (
				input,
				X);

			double errorResult = 0.0;

			for (int i = 0; i < errors.Length; i++) 
			{
				errorResult += errors [i];
			}

			errorResult = errorResult / errors.Length;

			return errorResult;
		}

        #endregion

        #region Private Methods

        private double[] calculateDelta(
            double[] a,
            double[] b)
        {
            if (a.Length < 0 ||
                b.Length < 0 ||
                b.Length != a.Length)
                throw new Exception("Different array size.");

            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
            {
                result[i] = -(a[i] - b[i]);
            }
            
            return result;
        }

        private double[] negateArray(
            double[] a)
        {
            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
            {
                result[i] = -a[i];
            }

            return result;
        }

        private double arraySquareModule(
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

        private double[] calculateDirection(
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

                result[i] = Xk1[i] + 
					bDirection;
                
				searchDirection[i] = bDirection -
                    deltaK[i];
				
				result[i] = ClampSolution.ClampX (input, result, i);
            }

            return result;
        }

		private double getMediumSquareErrorDiff(
			double[] a,
			double[] b)
		{
			double diff = 0.0;
			double buf;

			for (int i = 0; i < a.Length; i++) 
			{
				buf = a [i] - b [i];	
				diff += buf * buf;
			}

			return diff / a.Length;
		}

        #endregion
    }
}
