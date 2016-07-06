using System.Threading.Tasks;
using PhysicsEngineMathUtility;

namespace LCPSolver
{
    public class ProjectedGaussSeidel : ISolver
    {
		#region Fields

		double MSE;

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

			double internalSOR = SolverParameters.SOR;

			for (int k = 0; k < SolverParameters.MaxIteration; k++) 
			{
				double[] sum = lowerTriangularMatrix(input, X);

				for (int i = 0; i < input.Count; i++)
				{
					double sumBuffer = sum [i];

					SparseElement m = input.M [i];

					double[] bufValue = m.Value;
					int[] bufIndex = m.Index;

					//Avoid first row elaboration
					if (i != 0) 
					{
						for (int j = 0; j < m.Count; j++) 
						{
							if (bufIndex [j] < i) 
								sumBuffer += bufValue [j] * X [bufIndex [j]];
						}
					}

					sumBuffer = (input.B[i] - sumBuffer) * input.D[i];

					X[i] += (sumBuffer - X[i]) * internalSOR;

					sum[i] = sumBuffer;

					X[i] = ClampSolution.ClampX (input, X, i);

					diffX [i] = X [i] - oldX [i];
					oldX [i] = X [i];
				}
					
				MSE = getMediumSquareError (diffX);

				if (MSE < SolverParameters.ErrorTolerance)
					return X;
			}

            return X;
        }

        public void SetSuccessiveOverRelaxation(double SOR)
        {
			SolverParameters.SetSOR(SOR);
        }

		/// <summary>
		/// Gets the difference between second to last/last vector solution.
		/// </summary>
		/// <returns>The mse.</returns>
		public double GetDifferentialMSE()
		{
			return MSE;
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
				new ParallelOptions { MaxDegreeOfParallelism = SolverParameters.MaxThreadNumber }, 
				i => {
					sum [i] = kernel (input, X, i);
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
			double[] vector)
		{
			double buf = 0.0;

			foreach(double value in vector)
				buf += value * value;
			
			return buf / vector.Length;
		}
       
        #endregion
    }
}
