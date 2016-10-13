using PhysicsEngineMathUtility;
using SimulationObjectDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LCPSolver
{
    public class GMRES : ISolver
    {

        #region Fields

        ProjectedGaussSeidel gaussSeidelSolver;

        public readonly SolverParameters solverParam;

        #endregion

        #region Constructor

        public GMRES(
            SolverParameters solverParameters)
        {
            solverParam = solverParameters;

            var gaussSeidelSolverParam = new SolverParameters(
                                                          solverParameters.MaxIteration,
                                                          solverParameters.ErrorTolerance,
                                                          solverParameters.SOR,
                                                          solverParameters.MaxThreadNumber,
                                                          solverParameters.SORStep);

            gaussSeidelSolver = new ProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public double[] Solve(LinearProblemProperties input)
        {
            double[] X1 = gaussSeidelSolver.Solve(input);

            double testError = input.ComputeSolverError(X1);

            //Check for zero variable

            List<int> indexes = new List<int>();
            for (int i = 0; i < X1.Length; i++)
            {
                if (Math.Abs(X1[i]) < 1E-30)
                    indexes.Add(i);
            }

            if (indexes.Count > 0)
            {
                //Check input to delete
                for (int i = 0; i < indexes.Count; i++)
                {
                    if (input.ConstraintType[indexes[i]] == ConstraintType.Collision)
                    {
                        input.SetConstraintType(ConstraintType.None, indexes[i]);
                        input.SetConstraintType(ConstraintType.None, indexes[i] + 1);
                        input.SetConstraintType(ConstraintType.None, indexes[i] + 2);
                    }
                    else if(input.ConstraintType[indexes[i]] != ConstraintType.Friction)
                    {
                        input.SetConstraintType(ConstraintType.None, indexes[i]);
                    }
                }

                List<double> B = input.B.ToList();
                List<double> D = input.D.ToList();
                List<double> constraintsLimit = input.ConstraintLimit.ToList();
                List<ConstraintType> type = input.ConstraintType.ToList();
                List<int?> constraints = input.Constraints.ToList();
                List<List<double>> M = input.GetOriginalMatrixList();

                //Delete zero values
                for (int i = 0; i < indexes.Count; i++)
                {
                    for (int j = 0; j < type.Count; j++)
                    {
                        if (type[j] == ConstraintType.None)
                        {
                            B.RemoveAt(j);
                            D.RemoveAt(j);
                            constraintsLimit.RemoveAt(j);
                            constraints.RemoveAt(j);
                            type.RemoveAt(j);
                            M.RemoveAt(j);

                            for (int k = 0; k < M.Count(); k++)
                                M[k].RemoveAt(j);
                            
                            break;
                        }
                    }
                }

                //Build new input matrix
                SparseElement[] sparseM = BuildSparseMatrix(M);
                double[] X = new double[M.Count];

                LinearProblemProperties newInput = new LinearProblemProperties(
                    sparseM,
                    B.ToArray(),
                    X,
                    D.ToArray(),
                    constraintsLimit.ToArray(),
                    type.ToArray(),
                    constraints.ToArray(),
                    type.Count);

                double[] X2 = gaussSeidelSolver.Solve(newInput);
                
                //Union of two solutions
                int t = 0;
                double[] xTemp = new double[X1.Length];
                for (int i = 0; i < X1.Length; i++)
                {
                    if (X1[i] != 0)
                    {
                        xTemp[i] = X2[t];
                        t++;
                    }
                }

                double testError1 = input.ComputeSolverError(xTemp);

                if (testError1 < testError)
                {
                    Array.Copy(xTemp, X1, X1.Length);
                }

                Console.WriteLine("Error1 " + testError + " error2 " + testError1);
            }

            return X1;
        }

        #region ISolver

        public double GetDifferentialMSE()
        {
            throw new NotImplementedException();
        }

        public SolverParameters GetSolverParameters()
        {
            return gaussSeidelSolver.SolverParameters;
        }

        #endregion

        #endregion

        #region Private Methods

        private SparseElement[] BuildSparseMatrix(List<List<double>> matrix)
        {
            SparseElement[] sparseMatrix = new SparseElement[matrix.Count];

            List<int>[] index = new List<int>[matrix.Count];
            List<double>[] value = new List<double>[matrix.Count];

            for (int i = 0; i < matrix.Count; i++)
            {
                index[i] = new List<int>();
                value[i] = new List<double>();
            }

            for (int i = 0; i < matrix.Count; i++)
            {
                for (int j = i + 1; j < matrix.Count; j++)
                {
                    if (Math.Abs(matrix[i][j]) > 1E-30)
                    {
                        index[i].Add(j);
                        value[i].Add(matrix[i][j]);
                        index[j].Add(i);
                        value[j].Add(matrix[j][i]);
                    }
                }
            }

            for (int i = 0; i < matrix.Count; i++)
            {
                sparseMatrix[i] = new SparseElement(
                    value[i].ToArray(),
                    index[i].ToArray());
            }

            return sparseMatrix;
        }
        
        #endregion
    }
}
