using System;
using System.Threading.Tasks;
using PhysicsEngineMathUtility;

namespace LCPSolver
{
    public class ProjectedGaussSeidelBase
    {
		#region Fields

		public readonly SolverParameters SolverParameters;

		#endregion

        #region Constructor

        public ProjectedGaussSeidelBase(
			SolverParameters solverParameters)
        {
			SolverParameters = solverParameters;
        }

        #endregion

        #region Public Methods

        public double[] Solve(
            LinearProblemProperties input,
            double[][] matrix)
        {
			double[] X = new double[input.Count];
			double[] oldX = new double[input.Count];
			
			for (int i = 0; i < input.Count; i++) 
			{
				oldX[i] = X [i] = input.StartX [i];
			}

			double internalSOR = SolverParameters.SOR;

            	for (int k = 0; k < SolverParameters.MaxIteration; k++) 
			{
				double[] sum = ElaborateLowerTriangularMatrix(matrix, X);

                double mseDiff = 0.0;

				for (int i = 0; i < input.Count; i++)
				{
					double sumBuffer = sum [i];

                    double[] row = matrix[i];

					for (int j = 0; j < i; j++) 
					{
						sumBuffer += row [j] * X [j];
					}
					
					sumBuffer = (input.B[i] - sumBuffer) * input.D[i];

					X[i] += (sumBuffer - X[i]) * internalSOR;

					//X[i] = ClampSolution.ClampX (input, X, i);

                    sum[i] = sumBuffer;

                    double diff = X[i] - oldX[i];

                    oldX [i] = X [i];

                    mseDiff += diff * diff;
				}

                //if (mseDiff / input.Count < SolverParameters.ErrorTolerance)
                //    return X;

                
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
			return 0;
		}

		public SolverParameters GetSolverParameters()
		{
			return SolverParameters;
		}

        #endregion

        #region Private Methods

        private double[] ElaborateLowerTriangularMatrix(
            double[][] matrix,
            double[] X)
        {
            double[] sum = new double[matrix.Length];

            for (int i = 0; i < matrix.Length; i++)
            {
                sum[i] = kernel(matrix[i], X, i);
            }

			return sum;
        }

        private double kernel(
            double[] row,
            double[] X,
            int i)
        {
			double sumBuffer = 0.0;

			for (int j = i+1; j < row.Length; j++) {
                sumBuffer += row[j] * X[j];
			}
			
            return sumBuffer;
        }

		#endregion
    }
}
