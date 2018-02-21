using SharpEngineMathUtility;

namespace SharpPhysicsEngine.LCPSolver
{
    public static class SolverHelper
    {
        public static double ComputeSolverError(
            LinearProblemProperties input,
            double[] X)
        {
            double error = 0.0;

            for (int i = 0; i < input.Count; i++)
            {
                SparseElement m = input.M[i];

                double[] bufValue = m.Value;
                int[] bufIndex = m.Index;

                double bValue = (1.0 / input.InvD[i]) * X[i];

                for (int j = 0; j < m.Count; j++)
                    bValue += bufValue[j] * X[bufIndex[j]];

                error += (bValue - input.B[i]) * (bValue - input.B[i]);
            }

            return error;
        }
    }
}
