using SharpEngineMathUtility;
using static SharpEngineMathUtility.SparseElement;
using static SharpEngineMathUtility.GeneralMathUtilities;
using System;
using SharpPhysicsEngine.Solver;

namespace SharpPhysicsEngine.LCPSolver
{
    public sealed class ProjectedConjugateGradient : ISolver
    {
        #region Fields

        ProjectedGaussSeidel gaussSeidelSolver;
        public readonly SolverParameters SolverParameters;

        #endregion

        #region Constructor

        public ProjectedConjugateGradient(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;

            var gaussSeidelSolverParam = new SolverParameters(
                                                          2,
                                                          SolverParameters.ErrorTolerance,
                                                          1.0,
                                                          SolverParameters.MaxThreadNumber,
                                                          SolverParameters.SORStep,
                                                          false);

            gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
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

            double[] x = Array.ConvertAll(X, item => item.X);
           
            SparseElement[] A = linearProblemProperties.GetOriginalSparseMatrix();
            double[] r = GetDirection(A, linearProblemProperties.B, x);

            if (Dot(r, r) < 1E-50)
                return X;

            double[] p = r;

            //Solve Constraint without bound
            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                double alphaCG = GetAlphaCG(r, p, A);
                x = UpdateSolution(x, p, alphaCG);
                
                r = GetGradient(A, r, p, alphaCG);

                double[] phiY = GetPhi(linearProblemProperties, x, r);
                double[] partialValue = Multiply(A, p);
                double denom = Dot(p, partialValue);
                double beta = 0.0;
                if (denom != 0.0)
                    beta = Dot(phiY, partialValue) / denom;

                p = Minus(phiY, Multiply(beta, p));

                //Solve Constraint with bounds
                for (int j = 0; j < x.Length; j++)
                    X[j].X = x[j];

                SolutionValues[] y = gaussSeidelSolver.Solve(linearProblemProperties, X);

                x = Array.ConvertAll(y, item => item.X);
            }

           

            Console.WriteLine("Conjugate gradient error: " + Math.Sqrt(CheckErrorTest(x, A, linearProblemProperties)));

            for (int j = 0; j < x.Length; j++)
                X[j].X = x[j];

            return X;
        }

        #endregion

        #region Private Methods

        private double CheckErrorTest(double[] x, SparseElement[] A, LinearProblemProperties input)
        {
            double[] xValue = x;
            double[] dir = GetDirection(A, input.B, xValue);

            double error = 0.0;

            for (int i = 0; i < dir.Length; i++)
            {
                if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.Collision)
                    error += dir[i] * dir[i];
                else if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.Joint)
                    error += dir[i] * dir[i];

                else if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.SoftJoint)
                    error += dir[i] * dir[i];
                else if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.Friction)
                {
                    double? min = 0.0;
                    double? max = 0.0;

                    ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                    if (min.HasValue && xValue[i] + 1E-9 < min)
                        error += 0.0;
                    else if (max.HasValue && xValue[i] - 1E-9 > max)
                        error += 0.0;
                }
            }

            return error;
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
                return Dot(g, p) / den;

            return alphaCG;
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
                else
                    result[i] = 0.0;
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

        #endregion
    }
}
