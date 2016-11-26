using System;
using System.Threading.Tasks;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace LCPSolver
{
    public class ProjectedGaussSeidel : ISolver
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

        public SolutionValues[] Solve(
            LinearProblemProperties input,
            SolutionValues[] X = null)
        {
            if(X == null)
                X = new SolutionValues[input.Count];

			double internalSOR = SolverParameters.SOR;
            double solverError = double.MaxValue;

			for (int k = 0; k < SolverParameters.MaxIteration; k++) 
			{
				double[] sum = ElaborateLowerTriangularMatrix(input, X);

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
							if (bufIndex[j] < i) 
								sumBuffer += bufValue [j] * X [bufIndex[j]].X;
						}
					}

                    sumBuffer = (input.B[i] - sumBuffer) * input.D[i];

                    	X[i].X += (sumBuffer - X[i].X) * internalSOR;

					X[i] = ClampSolution.Clamp (input, X, i);

                    sum[i] = sumBuffer;
				}

                if (SolverParameters.DynamicSORUpdate)
                {
                    double actualSolverError = SolverHelper.ComputeSolverError(input, X);
                    
                    if (actualSolverError > solverError)
                        internalSOR = Math.Max(internalSOR - SolverParameters.SORStep, SolverParameters.SORStep);
                    else
                        solverError = actualSolverError;

                    if (actualSolverError < SolverParameters.ErrorTolerance)
                        return X;
                }
            }
             
            return X;
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

        private double[] ElaborateLowerTriangularMatrix(
            LinearProblemProperties input,
            SolutionValues[] X)
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
            SolutionValues[] X,
            int i)
        {
			double sumBuffer = 0.0;

			//Avoid last row elaboration
			if (i + 1 != input.Count)
            {
				double[] bufValue = input.M [i].Value;
				int[] bufIndex = input.M [i].Index;

				for (int j = 0; j < input.M [i].Count; j++) {
					if(bufIndex [j] > i) 
						sumBuffer += bufValue [j] * X [bufIndex [j]].X;
				}
			}
            return sumBuffer;
        }

        #endregion
    }
}
