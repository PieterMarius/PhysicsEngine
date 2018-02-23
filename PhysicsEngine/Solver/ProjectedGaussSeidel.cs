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
            for (int k = 0; k < SolverParameters.MaxIteration; k++) 
			{
                ElaborateUpperTriangularMatrix(input, ref x);
            }
            
            return x;
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
            ref double[] x)
        {
            double[] sum = ElaborateLowerTriangularMatrix(input, x);

            for (int i = 0; i < input.Count; i++)
            {
                double sumBuffer = sum[i];

                SparseElement m = input.M[i];

                //Avoid first row elaboration
                if (i != 0 && m.Count > 0)
                {
                    double[] bufValue = m.Value;
                    int[] bufIndex = m.Index;
                                                    
                    for (int j = 0; j < m.Count; j++)
                    {
                        int idx = bufIndex[j];
                        if (idx < i)
                            sumBuffer += bufValue[j] * x[idx];
                    }
                }

                sumBuffer = (input.B[i] - sumBuffer) * input.InvD[i];

                x[i] += (sumBuffer - x[i]) * SolverParameters.SOR;

                x[i] = ClampSolution.Clamp(input, x, i);
            }
        }

        private double[] ElaborateLowerTriangularMatrix(
            LinearProblemProperties input,
            double[] x)
        {
            double[] sum = new double[input.Count];

			Parallel.For (
                0, 
				input.Count, 
				new ParallelOptions { MaxDegreeOfParallelism = SolverParameters.MaxThreadNumber }, 
				i => {
					sum [i] = Kernel (input, x, i);
				});
				
            return sum;
        }

        private double Kernel(
            LinearProblemProperties input,
            double[] x,
            int i)
        {
			double sumBuffer = 0.0;

			//Avoid last row elaboration
			if (i + 1 != input.Count)
            {
                if (input.M[i].Count > 0)
                {
                    double[] bufValue = input.M[i].Value;
                    int[] bufIndex = input.M[i].Index;

                    for (int j = 0; j < input.M[i].Count; j++)
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
