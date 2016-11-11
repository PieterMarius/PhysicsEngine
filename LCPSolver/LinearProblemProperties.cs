using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace LCPSolver
{
	public class LinearProblemProperties
	{
		#region Properties

		//Matrice termini noti (N*N)
		public readonly SparseElement[] M;

		//Vettore dei valori attesi
		public readonly double[] B;

		//Diagonale matrice
		public readonly double[] D;

		//Vincoli
		public readonly double[] ConstraintLimit;

		//Tipo vincolo
		public ConstraintType[] ConstraintType { get; private set; }

		//Legame contatto
		public readonly int?[][] Constraints;

		//Dimensioni vettori
		public readonly int Count;

        	#endregion

		#region Constructor

		public LinearProblemProperties (
			SparseElement[] M,
			double[] B,
			SolutionValues[] startX,
			double[] d,
			double[] constraintLimit,
			ConstraintType[] constraintType,
			int?[][] constraints,
            	int count)
		{
			this.M = M;
			this.B = B;
			D = d;
			ConstraintLimit = constraintLimit;
			ConstraintType = constraintType;
			Constraints = constraints;
			Count = count;
		}

		#endregion

		#region Public Methods
        
		public double[][] GetOriginalMatrix()
		{
			double[][] matrix = new double[Count][];

			for (int i = 0; i < Count; i++)
			{
				matrix[i] = GetOriginalRow(i);
				matrix[i][i] = 1.0 / D[i];
			}

			return matrix;
		}

        public List<List<double>> GetOriginalMatrixList()
        {
            List<List<double>> matrix = new List<List<double>>();

            for (int i = 0; i < Count; i++)
            {
                matrix.Add(GetOriginalRow(i).ToList());
                matrix[i][i] = 1.0 / D[i];
            }

            return matrix;
        }

        public void SetConstraintType(ConstraintType type, int index)
        {
            ConstraintType[index] = type;
        }

        public double ComputeSolverError(
            double[] X)
        {
            double[][] matrix = GetOriginalMatrix();

            double error = 0.0;

            for (int i = 0; i < Count; i++)
            {
                double diffError = CalculateRowError(matrix[i], X, B[i]);
                
                error += diffError * diffError;
            }

            return error;
        }

        #endregion

        #region Private Methods

        private double[] GetOriginalRow(int index)
		{
			double[] row = new double[Count];

			SparseElement element = M[index];

			for (int i = 0; i < element.Count; i++)
				row[element.Index[i]] = element.Value[i];

			return row;
		}

        private double CalculateRowError(
            double[] row,
            double[] X,
            double expected)
        {
            double bValue = 0.0;
            for (int j = 0; j < Count; j++)
                bValue += row[j] * X[j];
            
            return bValue - expected;
        }

		#endregion

	}
}

