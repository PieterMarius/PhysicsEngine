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
				                                          1,
				                                          solverParam.ErrorTolerance,
				                                          solverParam.SOR,
				                                          solverParam.MaxThreadNumber,
				                                          solverParam.SORStep);
			
			gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public double[] Solve(LinearProblemProperties input)
        {
            double[] Xk = gaussSeidelSolver.Solve(input);
            double[] delta = calculateDelta(Xk, input.StartX);
            double[] searchDirection = negateArray(delta);

            input.SetStartValue(Xk);

            for (int i = 0; i < solverParam.MaxIteration; i++)
            {
                double[] Xk1 = gaussSeidelSolver.Solve(input);
                                
                double[] deltaK = calculateDelta(Xk1, Xk);

                deltaErrorCheck = arraySquareModule(delta);

				//early exit
				if (deltaErrorCheck < solverParam.ErrorTolerance)
					return Xk1;

                double betaK = 0.0;
				if (Math.Abs(deltaErrorCheck) > 10E-40)
                    betaK = arraySquareModule(deltaK) / deltaErrorCheck;

				if (betaK > 1.0)
					searchDirection = new double[searchDirection.Length];
                else
                {
					Xk = calculateDirection (
						input,
						Xk1, 
						deltaK, 
						ref searchDirection, 
						betaK);

                    input.SetStartValue(Xk);
                }

				Array.Copy(deltaK, delta, deltaK.Length);
            }
            return Xk;
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

                result[i] = Xk1[i] + bDirection;
                
				searchDirection[i] = bDirection - deltaK[i];
				
				result[i] = ClampSolution.ClampX (input, result, i);
            }

            return result;
        }

		#endregion
	}
}
