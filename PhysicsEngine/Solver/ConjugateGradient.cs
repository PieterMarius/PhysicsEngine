using SharpEngineMathUtility;
using static SharpEngineMathUtility.SparseElement;
using static SharpEngineMathUtility.GeneralMathUtilities;
using System;

namespace SharpPhysicsEngine.LCPSolver
{
    public sealed class ConjugateGradient : ISolver
    {
        #region Fields

        public readonly SolverParameters SolverParameters;

        #endregion

        #region Constructor

        public ConjugateGradient(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
        }

        public SolutionValues[] Solve(
            LinearProblemProperties linearProblemProperties, 
            SolutionValues[] X = null)
        {
            if (X == null)
                X = new SolutionValues[linearProblemProperties.Count];

            double[] x = new double[X.Length];
            for (int i = 0; i < x.Length; i++)
                x[i] = X[i].X;
            
            SparseElement[] A = linearProblemProperties.GetOriginalSparseMatrix();
            double[] g = GetDirection(A, linearProblemProperties.B, x);

            double a = 0.1;
            double t = 1.0;
                        
            if (Dot(g, g) < 1E-50)
                return X;

            double[] pPhi = GetPhi(linearProblemProperties, A, x, g);
            
            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                double[] sigma = GetPhi(linearProblemProperties, A, x, g);

                if (Dot(GetBetaS(linearProblemProperties, g, x, a), GetBeta(linearProblemProperties, g, x)) <= 
                    t * Dot(GetSigmaS(linearProblemProperties, sigma, x, a), sigma))
                {
                    ////Trial Conjugate Gradient Step
                    double alphaCG = GetAlphaCG(g, pPhi, A);
                    double[] y = UpdateSolution(x, pPhi, alphaCG);

                    double alphaf = GetAlphaF(linearProblemProperties, pPhi, x);

                    if(alphaCG < alphaf)
                    {
                        ////Conjugate Gradient Step
                        x = y;
                        g = GetGradient(A, g, pPhi, alphaCG);

                        double[] phiY = GetPhi(linearProblemProperties, A, y, g);
                        double[] partialValue = Multiply(A, pPhi);
                        double beta = Dot(phiY, partialValue) /
                                      Dot(pPhi, partialValue);

                        pPhi = Minus(phiY, Multiply(beta, pPhi));
                    }
                    else
                    {
                        ////Expansion Step
                        double[] partialx = UpdateSolution(x, pPhi, alphaf);
                        g = GetGradient(A, g, pPhi, alphaf);

                        x = GetSigmaPhi(linearProblemProperties, A, g, x, a);
                        x = Project(linearProblemProperties, x);

                        g = GetDirection(A, linearProblemProperties.B, x);
                        pPhi = GetPhi(linearProblemProperties, A, x, g);

                    }
                }
                else
                {
                    ////Proportioning Step preparation
                    double[] d = GetBeta(linearProblemProperties, g, x);
                    double alphaCG = GetAlphaCG(g, d, A);

                    double alphaf = GetAlphaF(linearProblemProperties, d, x);

                    if(alphaCG < alphaf)
                    {
                        ////Proportioning Step
                        x = Minus(x, Multiply(alphaCG, d));
                        g = GetGradient(A, g, d, alphaCG);

                        pPhi = GetPhi(linearProblemProperties, A, x, g);
                    }
                    else
                    {
                        ////Proportioning expansion step
                        double[] partialx = UpdateSolution(x, d, alphaf);
                        g = GetGradient(A, g, d, alphaf);

                        x = GetSigmaBeta(linearProblemProperties, A, g, x, a);
                        x = Project(linearProblemProperties, x);

                        g = GetDirection(A, linearProblemProperties.B, x);
                        pPhi = GetPhi(linearProblemProperties, A, x, g);
                    }
                }

                //x = Project(linearProblemProperties, x);
            }

            Console.WriteLine("Conjugate gradient error: " + Math.Sqrt(CheckErrorTest(x,A,linearProblemProperties)));

            for (int i = 0; i < x.Length; i++)
            {
                X[i].X = x[i];
                //X[i] = ClampSolution.Clamp(linearProblemProperties, X, i);
            }
            
            return X;
        }

        #endregion

        #region Private Methods

        private double CheckErrorTest(double[] x, SparseElement[] A, LinearProblemProperties input)
        {
            double[] dir = GetDirection(A, input.B, x);

            double error = 0.0;

            for (int i = 0; i < dir.Length; i++)
            {
                if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.Collision)
                    error += dir[i] * dir[i];
                else if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.Friction)
                {
                    double min = 0.0;
                    double max = 0.0;

                    ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                    if (x[i] + 1E-9 < min)
                        error += 0.0;
                    else if (x[i] - 1E-9 > max)
                        error += 0.0;
                }
            }

            return error;
        }

        private double[] GetPhi(
            LinearProblemProperties input,
            SparseElement[] A,
            double[] x,
            double[] g)
        {
            double[] result = new double[input.Count];

            for (int i = 0; i < input.Count; i++)
            {
                if (!ClampSolution.GetIfClamped(input, x, i))
                    result[i] = g[i];
                else
                {
                    result[i] = 0.0;
                    //double min = 0.0;
                    //double max = 0.0;
                    //ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                    //if (x[i] >= min && x[i] <= max)
                    //{
                    //    result[i] = g[i];
                    //}
                    //else
                    //{ 
                    //    result[i] = 0.0;
                    //}
                }
            }

            return result;
        }

        /// <summary>
        /// (g^t * p) / (p^t *A * p)
        /// </summary>
        /// <param name="g"></param>
        /// <param name="p"></param>
        /// <param name="A"></param>
        /// <returns></returns>
        private double GetAlphaCG(
            double[] g,
            double[] p,
            SparseElement[] A)
        {
            double den = Dot(Multiply(A, p), p);
            double alphaCG = 0.0;

            if (den != 0)
                alphaCG = Dot(g, p) / den;

            return alphaCG;
        }

        /// <summary>
        /// x - alpha * p
        /// </summary>
        /// <param name="x"></param>
        /// <param name="p"></param>
        /// <param name="alpha"></param>
        /// <returns></returns>
        private double[] UpdateSolution(
            double[] x,
            double[] p,
            double alpha)
        {
            return Minus(x, Multiply(alpha, p));
        }

        /// <summary>
        /// g - alpha * A * p
        /// </summary>
        /// <param name="A"></param>
        /// <param name="g"></param>
        /// <param name="p"></param>
        /// <param name="alpha"></param>
        /// <returns></returns>
        private double[] GetGradient(
            SparseElement[] A,
            double[] g,
            double[] p,
            double alpha)
        {
            return Minus(g, Multiply(alpha, Multiply(A, p)));
        }

        /// <summary>
        /// Ax - b
        /// </summary>
        /// <param name="A"></param>
        /// <param name="b"></param>
        /// <param name="x"></param>
        /// <returns></returns>
        private double[] GetDirection(
            SparseElement[] A,
            double[] b,
            double[] x)
        {
            return Minus(Multiply(A, x), b);
        }

        private double GetAlphaF(
            LinearProblemProperties input,
            double[] p,
            double[] x)
        {
            double outputMin = double.MaxValue;

            for (int i = 0; i < input.Count; i++)
            {
                double min = 0.0;
                double max = 0.0;
                double bufMin = double.MaxValue;
                ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                if (p[i] > 0.0)
                    bufMin = (x[i] - min) / p[i];
                else if (p[i] < 0.0)
                    bufMin = (x[i] - max) / p[i];

                if(bufMin < outputMin)
                    outputMin = bufMin;
            }

            return (outputMin == double.MaxValue) ? 0.0 : outputMin;
        }

        private double[] GetSigmaPhi(
            LinearProblemProperties input,
            SparseElement[] A,
            double[] g,
            double[] x,
            double a)
        {
            return Minus(x, Multiply(a, GetPhi(input, A, x, g)));
        }

        private double[] GetSigmaBeta(
            LinearProblemProperties input,
            SparseElement[] A,
            double[] g,
            double[] x,
            double a)
        {
            return Minus(x, Multiply(a, GetBeta(input, g, x)));
        }

        private double[] GetBeta(
            LinearProblemProperties input,
            double[] g,
            double[] x)
        {
            double[] output = new double[x.Length];

            for (int i = 0; i < input.Count; i++)
            {
                double lower = 0.0;
                double upper = 0.0;
                ClampSolution.GetConstraintValues(input, x, i, ref lower, ref upper);

                if (Math.Abs(x[i] - lower) < 1E-50)
                    output[i] = Math.Min(0.0, g[i]);
                else if (Math.Abs(x[i] - upper) < 1E-50)
                    output[i] = Math.Max(0.0, g[i]);
            }

            return output;
        }

        private double[] GetBetaS(
            LinearProblemProperties input,
            double[] g,
            double[] x,
            double a)
        {
            double[] output = new double[x.Length];

            for (int i = 0; i < input.Count; i++)
            {
                double lower = 0.0;
                double upper = 0.0;
                ClampSolution.GetConstraintValues(input, x, i, ref lower, ref upper);

                if (Math.Abs(x[i] - lower) < 1E-50 && 
                    g[i] < 0.0)
                    output[i] = Math.Min(0.0, g[i]);
                else if (Math.Abs(x[i] - upper) < 1E-50 && 
                        g[i] > 0.0)
                    output[i] = Math.Max(0.0, g[i]);
                else
                    output[i] = 0.0;
            }

            return output;
        }

        private double[] GetSigmaS(
            LinearProblemProperties input,
            double[] sigmaValues,
            double[] x,
            double a)
        {
            double[] output = new double[x.Length];

            for (int i = 0; i < input.Count; i++)
            {
                double lower = 0.0;
                double upper = 0.0;
                ClampSolution.GetConstraintValues(input, x, i, ref lower, ref upper);

                if (sigmaValues[i] > 0.0)
                    output[i] = Math.Min((x[i] - lower) / a, sigmaValues[i]);
                else if (sigmaValues[i] < 0.0)
                    output[i] = Math.Max((x[i] - upper) / a, sigmaValues[i]);
            }

            return output;
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

        #endregion
    }
}
