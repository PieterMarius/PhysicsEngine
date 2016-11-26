using System;

namespace LCPSolver
{
	public class NonLinearConjugateGradient : ISolver
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
				                                          3,
				                                          solverParam.ErrorTolerance,
				                                          1.0,
				                                          solverParam.MaxThreadNumber,
				                                          solverParam.SORStep,
                                                          false);
			
			gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public SolutionValues[] Solve(
            LinearProblemProperties input,
            SolutionValues[] X = null)
        {
            if (X == null)
                X = new SolutionValues[input.Count];

            SolutionValues[] Xk = gaussSeidelSolver.Solve(input);
            double[] delta = calculateDelta(Xk, X);
            double[] searchDirection = negateArray(delta);

            SolutionValues[] Xk1 = new SolutionValues[input.Count];

            for (int i = 0; i < solverParam.MaxIteration; i++)
            {
                Xk1 = gaussSeidelSolver.Solve(input, Xk);
                                
                double[] deltaK = calculateDelta(Xk1, Xk);

                deltaErrorCheck = arraySquareModule(delta);
                
                double betaK = 1.1;
				if (Math.Abs(deltaErrorCheck) > 1E-40)
                    betaK = arraySquareModule(deltaK) / deltaErrorCheck;

				if (betaK > 1.0)
					searchDirection = new double[searchDirection.Length];
                else
                {
					Xk1 = calculateDirection (
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

        private double[] calculateDelta(
            SolutionValues[] a,
            SolutionValues[] b)
        {
			if (a.Length < 0 ||
				b.Length < 0 ||
				b.Length != a.Length)
			{
				throw new Exception("Different array size.");
			}

            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                result[i] = -(a[i].X - b[i].X);
            
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

        private SolutionValues[] calculateDirection(
			LinearProblemProperties input,
			SolutionValues[] Xk1,
            double[] deltaK,
            ref double[] searchDirection,
            double betak)
        {
            SolutionValues[] result = new SolutionValues[Xk1.Length];

            for (int i = 0; i < Xk1.Length; i++)
            {
                double bDirection = betak * searchDirection[i];

                result[i].X = Xk1[i].X + bDirection;

                searchDirection[i] = bDirection - deltaK[i];

                result[i] = ClampSolution.Clamp(input, result, i);
            }

            return result;
        }

		#endregion
	}
}
