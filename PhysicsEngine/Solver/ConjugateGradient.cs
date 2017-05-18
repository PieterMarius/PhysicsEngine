using PhysicsEngineMathUtility;
using static PhysicsEngineMathUtility.GeneralMathUtilities;

namespace SharpPhysicsEngine.LCPSolver
{
    public class ConjugateGradient : ISolver
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
            
            SparseElement[] A = linearProblemProperties.GetOriginalMatrixSparse();
            double[] g = Minus(Multiply(A, x), linearProblemProperties.B);
                        
            if (Dot(g, g) < 1E-50)
                return X;

            double[] pPhi = GetPhi(linearProblemProperties, A, x, g);
            
            for (int i = 0; i < SolverParameters.MaxIteration; i++)
            {
                double alphaKDen = Dot(Multiply(A, pPhi), pPhi);
                double alphaK = 0.0;
                if (alphaKDen != 0)
                    alphaK = Dot(g, pPhi) / alphaKDen;
                else
                {
                    string ciao = "ciao";
                }

                double[] halfX = Minus(x, Multiply(alphaK, pPhi));
                x = Project(linearProblemProperties, halfX);
                
                g = Minus(Multiply(A, x), linearProblemProperties.B);
                pPhi = GetPhi(linearProblemProperties, A, x, g);
            }

            for (int i = 0; i < x.Length; i++)
            {
                X[i].X = x[i];
                X[i] = ClampSolution.Clamp(linearProblemProperties, X, i);
            }
            
            return X;
        }

        #endregion

        #region Private Methods

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
                    double min = 0.0;
                    double max = 0.0;
                    ClampSolution.GetConstraintValues(input, x, i, ref min, ref max);

                    if (x[i] >= min && x[i] <= max)
                    {
                        result[i] = g[i];
                    }
                    else
                    { 
                        result[i] = 0.0;
                    }
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

        #endregion
    }
}
