using PhysicsEngineMathUtility;
using ShapeDefinition;
using System.Collections.Generic;
using System.Linq;

namespace LCPSolver
{
	public class LinearProblemProperties
	{
		#region Properties

        //y = Ax + B 

		//Matrix A (N*N)
		public readonly SparseElement[] M;

		//Vector B (N)
		public readonly double[] B;

		//Diagonal of matrix A (N)
		public readonly double[] D;

		//Constraint
		public readonly double[] ConstraintLimit;

		//Contraint type
		public ConstraintType[] ConstraintType { get; private set; }

		//Constraint joint
		public readonly int?[][] Constraints;

		//Size of the system
		public readonly int Count;

        	#endregion

		#region Constructor

		public LinearProblemProperties (
			SparseElement[] M,
			double[] B,
			double[] D,
			double[] constraintLimit,
			ConstraintType[] constraintType,
			int?[][] constraints)
		{
			this.M = M;
			this.B = B;
			this.D = D;
			ConstraintLimit = constraintLimit;
			ConstraintType = constraintType;
			Constraints = constraints;
			Count = B.Length;
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

        public SparseElement[] GetOriginalMatrixSparse()
        {
            SparseElement[] originalMatrix = new SparseElement[M.Length];

            for (int i = 0; i < M.Length; i++)
            {
                List<double> valueList = M[i].Value.ToList();
                List<int> indexList = M[i].Index.ToList();

                valueList.Add(1.0 / D[i]);
                indexList.Add(i);

                originalMatrix[i] = new SparseElement(valueList.ToArray(), indexList.ToArray(), M[i].RowLength);
            }

            return originalMatrix;
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

