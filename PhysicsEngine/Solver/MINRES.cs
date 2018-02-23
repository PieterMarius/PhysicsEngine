using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using static SharpEngineMathUtility.GeneralMathUtilities;
using static SharpEngineMathUtility.SparseElement;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class MINRES : ISolver
    {
        #region Fields

        ProjectedGaussSeidel gaussSeidelSolver;
        public readonly SolverParameters SolverParameters;

        #endregion

        #region Constructor

        public MINRES(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;

            var gaussSeidelSolverParam = new SolverParameters(
                                                          1,
                                                          SolverParameters.ErrorTolerance,
                                                          1.0,
                                                          SolverParameters.MaxThreadNumber);

            gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
        }

        private double[] GetDirection(
            SparseElement[] A,
            double[] b,
            double[] x)
        {
            return Minus(Multiply(A, x), b);
        }

        public double[] Solve(
            LinearProblemProperties linearProblemProperties, 
            double[] start_x)
        {
            double[] x = start_x;

            SparseElement[] A = linearProblemProperties.GetOriginalSparseMatrix();
            double[] v0 = new double[x.Length];
            double[] v1 = Minus(linearProblemProperties.B, Multiply(A, x));

            if (Dot(v1, v1) < 1E-50)
                return x;

            double beta1 = Math.Sqrt(Dot(v1, v1));
            double betaN = 0.0;
            double n = beta1;
            double c0 = 1.0;
            double c1 = 1.0;
            double s0 = 0.0;
            double s1 = 0.0;
            double[] w0 = new double[v1.Length];
            double[] w_1 = new double[v1.Length];
            double residual = double.NaN;
            
            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                //Calculate Lanczos Vectors
                double[] v = Multiply((1.0 / beta1), v1);
                double[] Av = Multiply(A, v);
                double alpha = Dot(v, Av);
                v1 = Minus(Minus(Av, Multiply(alpha, v)), Multiply(beta1, v0));
                betaN = Math.Sqrt(Dot(v1, v1));

                //Calculate QR factors
                double lambda = c1 * alpha - c0 * s1 * beta1;
                double p1 = Math.Sqrt(lambda * lambda + betaN * betaN);
                double p2 = s1 * alpha + c0 * c1 * beta1;
                double p3 = s0 * beta1;

                //Calculate New Givens Rotations
                c0 = c1;
                c1 = lambda / p1;

                s0 = s1;
                s1 = betaN / p1;

                //Update Solution
                double[] w = Multiply((1.0 / p1), (Minus(Minus(v, Multiply(p3, w_1)), Multiply(p2, w0))));

                x = Plus(x, Multiply(c1, Multiply(n, w)));
                x = gaussSeidelSolver.Solve(linearProblemProperties, x);
                //v1 = Minus(linearProblemProperties.B, Multiply(A, x));
                //x = Project(linearProblemProperties, x);

                n = -s1 * n;

                residual = Math.Abs(n);

                //if (residual < precisionConst)
                //    break;

                beta1 = betaN;
                v0 = v;
                w_1 = w0;
                w0 = w;
            }

            //x = gaussSeidelSolver.Solve(linearProblemProperties, x);

            Console.WriteLine("Conjugate gradient error: " + Math.Sqrt(CheckErrorTest(x, A, linearProblemProperties)));

            return x;
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
                if (input.ConstraintType[i] == ConstraintType.Collision)
                    error += dir[i] * dir[i];
                else if (input.ConstraintType[i] == ConstraintType.Joint)
                    error += dir[i] * dir[i];

                else if (input.ConstraintType[i] == ConstraintType.SoftJoint)
                    error += dir[i] * dir[i];
                //else if (input.ConstraintType[i] == ShapeDefinition.ConstraintType.Friction)
                //{
                //    double? min = 0.0;
                //    double? max = 0.0;

                //    ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                //    if (min.HasValue && xValue[i] + 1E-9 < min)
                //        error += 0.0;
                //    else if (max.HasValue && xValue[i] - 1E-9 > max)
                //        error += 0.0;
                //}
            }

            return error;
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
