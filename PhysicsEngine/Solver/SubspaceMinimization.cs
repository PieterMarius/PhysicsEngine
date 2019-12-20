using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Solver
{
    internal class SubspaceMinimization : ISolver
    {
        #region Private Fields

        private class SubspaceValues
        {
            public List<int> LowerIndexes;
            public List<int> UpperIndexes;
            public List<int> UnboundedIndexes;

            public SubspaceValues()
            {
                LowerIndexes = new List<int>();
                UpperIndexes = new List<int>();
                UnboundedIndexes = new List<int>();
            }
        }

        ProjectedGaussSeidel gaussSeidelSolver;

        public readonly SolverParameters solverParam;

        #endregion

        #region Constructor

        public SubspaceMinimization(SolverParameters solverParameters)
        {
            solverParam = solverParameters;

            var gaussSeidelSolverParam = new SolverParameters(
                                                          25,
                                                          solverParam.ErrorTolerance,
                                                          solverParam.SOR,
                                                          solverParam.MaxThreadNumber);

            gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            throw new NotImplementedException();
        }

        public double[] Solve(LinearProblemProperties input, JacobianConstraint[] constraints, double[] x)
        {
            x = gaussSeidelSolver.Solve(input, constraints, x);



            return x;
        }

        #endregion

        #region Private Methods

        private SubspaceValues GetIndexSets(LinearProblemProperties input, double[] x)
        {
            var result = new SubspaceValues();
            
            for (int i = 0; i < x.Length; i++)
            {
                switch (input.ConstraintType[i])
                {
                    case ConstraintType.Collision:
                    case ConstraintType.JointLimit:
                        if (x[i] <= 0.0)
                            result.LowerIndexes.Add(i);
                        else
                            result.UnboundedIndexes.Add(i);
                        break;

                    case ConstraintType.Friction:
                        int sign = (input.Constraints[i].Value < 0) ? -1 : 1;
                        int normalIndex = sign * input.Constraints[i].Value;
                        double frictionLimit = x[normalIndex] * input.ConstraintLimit[i];
                        int idx1 = normalIndex + sign;
                        int idx2 = normalIndex + sign * 2;
                                                
                        double directionA = x[idx1];
                        double directionB = x[idx2];
                        double frictionValue = Math.Sqrt(directionA * directionA + directionB * directionB);

                        if (frictionValue > frictionLimit)
                        {
                            Vector2d frictionNormal = new Vector2d(directionA / frictionValue, directionB / frictionValue);
                        }

                        break;
                    default:
                        break;
                }
            }

            return result;
        }

        #endregion
    }
}
