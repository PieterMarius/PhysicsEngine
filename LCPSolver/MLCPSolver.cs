using System;
using PhysicsEngineMathUtility;
using static PhysicsEngineMathUtility.GeneralMathUtilities;

namespace LCPSolver
{
    public class MLCPSolver : ISolver
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

            if(X == null)
                X = new SolutionValues[linearProblemProperties.Count];

            double[] x = new double[X.Length];
            for (int i = 0; i < x.Length; i++)
                x[i] = X[i].X;

            SparseElement[] A = linearProblemProperties.GetOriginalMatrixSparse();
            double[] g = Minus(Multiply(A, x), linearProblemProperties.B);

            if (Dot(g, g) < 1E-50)
                return X;

            double[] p = GetPhi(linearProblemProperties, x, g);

            double alphaCGDen = 1.0;
            double alphaCG = 0;
            double alphaTest = 1.6 / Math.Sqrt((Dot(linearProblemProperties.D, linearProblemProperties.D)));
            double gamma = 1.0;
            
            #endregion

            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                double[] betaf = BetaF(linearProblemProperties, g, x);
                double[] signedBetaF = SignedBetaF(linearProblemProperties, g, x, alphaTest);

                double betaTot = Dot(signedBetaF, betaf);

                double[] basePhi = GetPhi(linearProblemProperties, x, g);
                double[] signedPhi = GetSignedPhi(linearProblemProperties, x, basePhi, alphaTest);

                double phiTot = Dot(signedPhi, basePhi);

                if (betaTot <= gamma * phiTot)
                {
                    alphaCGDen = Dot(Multiply(A, p), p);
                    if (alphaCGDen != 0.0)
                        alphaCG = Dot(g, p) / alphaCGDen;

                    double[] y = Minus(x, Multiply(alphaCG, p));
                    double alphaF = GetMinAlphaF(linearProblemProperties, p, x);

                    if (alphaCG < alphaF)
                    {
                        //Conjugate gradient step
                        Array.Copy(y, x, x.Length);
                        g = Minus(g, Multiply(alphaCG, Multiply(A, p)));
                        double[] bufPhi = GetPhi(linearProblemProperties, x, g);
                        double beta = Dot(Multiply(A, bufPhi), p) / alphaCGDen;
                        p = Minus(bufPhi, Multiply(beta, p));
                    }
                    else
                    {
                        //Expansion step
                        double[] bufx = Minus(x, Multiply(alphaF, p));
                        g = Minus(g, Multiply(alphaF, Multiply(A, p)));
                        double[] bufPhi = GetPhi(linearProblemProperties, bufx, g);
                        x = Minus(bufx, Multiply(alphaTest, bufPhi));
                        x = Project(linearProblemProperties, x);
                        g = Minus(Multiply(A, x), linearProblemProperties.B);
                        p = GetPhi(linearProblemProperties, x, g);

                    }
                }
                else
                {
                    double[] d = BetaF(linearProblemProperties, g, x);
                    alphaCGDen = Dot(Multiply(A, d), d);
                    if (alphaCGDen != 0.0)
                        alphaCG = Dot(g, d) / alphaCGDen;
                    double alphaF = GetMinAlphaF(linearProblemProperties, d, x);

                    if (alphaCG < alphaF)
                    {
                        //Proportioning step
                        x = Minus(x, Multiply(alphaCG, d));
                        g = Minus(g, Multiply(alphaCG, Multiply(A, d)));
                        p = GetPhi(linearProblemProperties, x, g);
                    }
                    else
                    {
                        //Proportioning-expansion step
                        double[] bufx = Minus(x, Multiply(alphaF, d));
                        g = Minus(g, Multiply(alphaF, Multiply(A, d)));
                        double[] bufBeta = BetaF(linearProblemProperties, bufx, g);
                        x = Minus(bufx, Multiply(alphaTest, bufBeta));
                        x = Project(linearProblemProperties, x);
                        g = Minus(Multiply(A, x), linearProblemProperties.B);
                        p = GetPhi(linearProblemProperties, x, g);
                    }
                }
            }

            for (int i = 0; i < x.Length; i++)
                X[i].X = x[i];

            return X;
        }

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
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
                if (!ClampSolution.GetIfClamped(input, x, i))
                    result[i] = g[i];
            }

            return result;
        }

        private double[] GetSignedPhi(
            LinearProblemProperties input,
            double[] x,
            double[] basePhi,
            double alphaTest)
        {
            double[] result = new double[input.Count];

            for (int i = 0; i < input.Count; i++)
            {
                double min = double.MinValue;
                double max = double.MaxValue;
                ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                if (basePhi[i] > 0.0)
                    result[i] = Math.Min((x[i] - min) / alphaTest, basePhi[i]);
                else if (basePhi[i] < 0.0)
                    result[i] = Math.Max((x[i] - max) / alphaTest, basePhi[i]);
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
                else if(p[i] < 0.0)
                {
                    buf = (x[i] - max) / p[i];
                    if (buf < result)
                        result = buf;
                }
            }
            return result;
        }

        private double[] Project(
            LinearProblemProperties input,
            double[] x)
        {
            double[] result = new double[input.Count];

            for (int i = 0; i < input.Count; i++)
            {
                result[i] = ClampSolution.Clamp(input, x, i);
            }

            return result;
        }

        private double[] BetaF(
            LinearProblemProperties input,
            double[] g,
            double[] x)
        {
            double[] result = new double[x.Length];

            for (int i = 0; i < x.Length; i++)
            {
                double min = double.MinValue;
                double max = double.MaxValue;
                ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                if (Math.Abs(x[i] - min) < 1E-50)
                    result[i] = Math.Min(0, g[i]);
                else if (Math.Abs(x[i] - max) < 1E-50)
                    result[i] = Math.Max(0, g[i]);
                
            }
            return result;
        }

        private double[] SignedBetaF(
            LinearProblemProperties input,
            double[] g,
            double[] x,
            double alphaTest)
        {
            double[] result = new double[x.Length];

            for (int i = 0; i < x.Length; i++)
            {
                double min = double.MinValue;
                double max = double.MaxValue;
                ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                if (Math.Abs(x[i] - min) < 1E-50 && g[i] < 0.0)
                    result[i] = Math.Max((x[i] - max) / alphaTest, g[i]);
                else if (Math.Abs(x[i] - max) < 1E-50 && g[i] > 0.0)
                    result[i] = Math.Max((x[i] - min) / alphaTest, g[i]);

            }
            return result;
        }

        #endregion
    }
}
