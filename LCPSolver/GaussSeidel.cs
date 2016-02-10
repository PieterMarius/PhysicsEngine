using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PhysicsEngineMathUtility;

namespace LCPSolver
{
    public class GaussSeidel : ISolver
    {
        // Successive over relaxation term
		private double SOR;

		public readonly SolverParameters solverParameters;

        #region Constructor

        public GaussSeidel(
			SolverParameters solverParameters)
        {
			this.solverParameters = solverParameters;
			this.SOR = this.solverParameters.SOR;
        }

        #endregion

        #region Public Methods

        public double[] Solve(LinearProblemProperties input)
        {
			double[] X = new double[input.Count];
			double[] oldX = new double[input.Count];
			double[] diffX = new double[input.Count];

			for (int i = 0; i < input.Count; i++) 
			{
				X [i] = input.StartX [i];
				oldX [i] = X [i];
			}

			double internalSOR = this.SOR;

			for (int k = 0; k < this.solverParameters.MaxIteration; k++) 
			{
				double[] sum = this.lowerTriangularMatrix(input, X);

				for (int i = 0; i < input.Count; i++)
				{
					double sumBuffer = sum[i];

					double[] bufValue = input.M [i].Value;
					int[] bufIndex = input.M [i].Index;

					//Avoid first row elaboration
					if (i != 0) 
					{
						for (int j = 0; j < input.M [i].Count; j++) 
						{
							if (bufIndex [j] < i) 
							{
								sumBuffer += bufValue [j] * X [bufIndex [j]];
							}
						}
					}

					sumBuffer = (input.B[i] - sumBuffer) * input.D[i];

					X[i] += (sumBuffer - X[i]) * internalSOR;

					sum[i] = sumBuffer;

					X[i] = ClampSolution.ClampX (input, X, i);

					diffX [i] = X [i] - oldX [i];
					oldX [i] = X [i];
				}
					
				double error = this.getMediumSquareError (diffX);

				if (error < this.solverParameters.ErrorTolerance) 
					return X;
			}
				
            return X;
        }

        public void SetSuccessiveOverRelaxation(double SOR)
        {
            this.SOR = SOR;
        }

		public double[] GetError(
			LinearProblemProperties input,
			double[] X)
		{
			if (input.Count > 0) {
				double[] result = new double[input.Count];
				for (int i = 0; i < input.Count; i++) {
					result [i] = 0.0;
					for (int j = 0; j < input.M[i].Count; j++) {
						result [i] += input.M [i].Value[j] * X [input.M [i].Index[j]];
					}
					result [i] = (input.B [i] - result [i]);
					result [i] = result [i] * result [i];
				}

				return result;
			} else {
				throw new Exception ("Empty solver parameters");
			}
		}

		public double GetMediumSquareError(
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

        private double[] lowerTriangularMatrix(
            LinearProblemProperties input,
            double[] X)
        {
            double[] sum = new double[input.Count];

			Parallel.For (0, 
				input.Count, 
				new ParallelOptions { MaxDegreeOfParallelism = this.solverParameters.MaxThreadNumber }, 
				i => {
					sum [i] = this.kernel (input, X, i);
				});
				
            return sum;
        }

        private double kernel(
            LinearProblemProperties input,
            double[] X,
            int i)
        {
            double sumBuffer = 0.0;

			//Avoid last row elaboration
			if (i + 1 != input.Count) {

				double[] bufValue = input.M [i].Value;
				int[] bufIndex = input.M [i].Index;

				for (int j = 0; j < input.M [i].Count; j++) {
					if (bufIndex [j] > i) {
						sumBuffer += bufValue [j] * X [bufIndex [j]];
					}
				}
			}
            return sumBuffer;
        }

		private double getMediumSquareError(
			double[] diff)
		{
			double buf = 0.0;
			double delta;

			for (int i = 0; i < diff.Length; i++) 
			{
				delta = diff [i];
				buf += delta * delta;
			}

			return buf / diff.Length;
		}
       
        #endregion
    }
}
