using System;
using PhysicsEngineMathUtility;
using static PhysicsEngineMathUtility.GeneralMathUtilities;

namespace LCPSolver
{
    class MLCPSolver : ISolver
    {
        #region Fields

        public readonly SolverParameters SolverParameters;

        #endregion

        #region Constructor

        public MLCPSolver(
            SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;
        }

        #endregion

        #region Public Methods

        public SolutionValues[] Solve(
            LinearProblemProperties linearProblemProperties, 
            SolutionValues[] X = null)
        {
            #region Compute starting values

            if(X != null)
                X = new SolutionValues[linearProblemProperties.Count];

            double[] x = new double[X.Length];
            for (int i = 0; i < x.Length; i++)
                x[i] = X[i].X;

            SparseElement[] A = linearProblemProperties.GetOriginalMatrixSparse();
            double[] g = Minus(Multiply(A, x), linearProblemProperties.B);
            double[] p = GetPhi(linearProblemProperties, x, g);

            double alphaCGDen = 1.0;
            double alphaCG = 0;

            #endregion

            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                alphaCGDen = Dot(Multiply(A, p), p);
                alphaCG = Dot(g, p) / alphaCGDen;

                double[] y = Minus(x, Multiply(alphaCG, p));
                double alphaF = GetMinAlphaF(linearProblemProperties, p, x);

                if(alphaCG < alphaF)
                {
                    Array.Copy(y, x, x.Length);
                    g = Minus(g, Multiply(A, p));
                    double[] bufPhi = GetPhi(linearProblemProperties, x, g);
                    double beta = Dot(Multiply(A, bufPhi), p) / alphaCGDen;
                    p = Minus(bufPhi, Multiply(beta, p));
                }
                else
                {
                    double[] bufx = Minus(x, Multiply(alphaF, p));
                    g = Minus(g, Multiply(alphaF, Multiply(A, p)));
                    //TODO proietto la soluzione tra min e max
                }
            }

            return X;
        }

        public SolverParameters GetSolverParameters()
        {
            throw new NotImplementedException();
        }

        #endregion

        #region Private Methods

        private double[] GetPhi(
            LinearProblemProperties input,
            double[] x,
            double[] g)
        {
            double[] result = new double[input.Count];

            for (int i = 0; i < input.Count; i++)
            {
                if (ClampSolution.GetIfClamped(input, x, i))
                    result[i] = g[i];
            }

            return result;
        }

        private double GetMinAlphaF(
            LinearProblemProperties input,
            double[] p,
            double[] x)
        {
            double result = double.MaxValue;

            for (int i = 0; i < p.Length; i++)
            {
                double min = double.MinValue;
                double max = double.MaxValue;
                ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);
                
                double buf = 0.0;
                if(p[i] > 0.0)
                {
                    buf = (x[i] - min) / p[i];
                    if (buf < result)
                        result = buf;
                }
                else
                {
                    buf = (x[i] - max) / p[i];
                    if (buf < result)
                        result = buf;
                }
            }
            return result;
        }

        #endregion
    }
}
