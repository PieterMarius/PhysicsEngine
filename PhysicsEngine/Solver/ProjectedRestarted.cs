using SharpEngineMathUtility;
using SharpEngineMathUtility.Solver;
using System;
using static SharpEngineMathUtility.SparseMatrix;
using static SharpEngineMathUtility.MathUtils;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class ProjectedRestarted : ISolver
    {
        #region Fields

        public readonly SolverParameters SolverParameters;
        private readonly GMRES solver;
        private readonly RedBlackProjectedGaussSeidel rbSolver;

        #endregion

        #region Constructor

        public ProjectedRestarted(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;
            solver = new GMRES();

            var gaussSeidelSolverParam = new SolverParameters(
                                                          1,
                                                          SolverParameters.ErrorTolerance,
                                                          1.0,
                                                          SolverParameters.MaxThreadNumber);

            rbSolver = new RedBlackProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
        }

        public double[] Solve(LinearProblemProperties linearProblemProperties, double[] x)
        {
            var A = linearProblemProperties.GetOriginalSparseMatrix();
            var b = linearProblemProperties.B;
            double[] r = GetDirection(A, b, x);

            var results = new double[x.Length];
            Array.Copy(x, results, x.Length);
            double actualSolverError = 0.0;
            double[] oldX = new double[x.Length];
            Array.Copy(x, oldX, x.Length);

            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                var w = solver.Solve(A, r, new double[r.Length], 100, r.Length);
                results = Add(results, Multiply(1.0, w));
                Clamp(linearProblemProperties, ref results);
                results = rbSolver.Solve(linearProblemProperties, results);
                //Clamp(linearProblemProperties, ref results);

                actualSolverError = SolverHelper.ComputeSolverError(results, oldX);

                if (actualSolverError < SolverParameters.ErrorTolerance)
                    return results;

                Array.Copy(results, oldX, results.Length);
                //double errp1 = CheckErrorTest(results, A, linearProblemProperties);

                r = GetDirection(A, b, results);
            }

            

            return results;
        }

        #endregion

        #region Private Methods

        private double CheckErrorTest(double[] x, SparseMatrix A, LinearProblemProperties input)
        {
            double[] xValue = x;
            double[] dir = GetDirection(A, input.B, xValue);

            double error = 0.0;

            for (int i = 0; i < dir.Length; i++)
            {
                if (input.ConstraintType[i] == ConstraintType.Collision)
                {
                    //error += dir[i] * dir[i];
                }
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

        private double[] GetDirection(
            SparseMatrix A,
            double[] b,
            double[] x)
        {
            //return Minus(Multiply(A, x, SolverParameters.MaxThreadNumber), b);
            return Minus(b, Multiply(A, x, SolverParameters.MaxThreadNumber));
        }

        private void Clamp(
            LinearProblemProperties linearProblemProperties,
            ref double[] x)
        {
            for (int i = 0; i < linearProblemProperties.Count; i++)
            {

                ClampSolution.Clamp(linearProblemProperties, x[i], ref x, i);
                //if (x[i] < 0.0)
                //    x[i] = 0.0;
            }
        }

        #endregion
    }
}
