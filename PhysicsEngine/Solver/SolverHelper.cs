using PhysicsEngineMathUtility;

namespace SharpPhysicsEngine.LCPSolver
{
    public static class SolverHelper
    {
        public static double ComputeSolverError(
            LinearProblemProperties input,
            SolutionValues[] X)
        {
            double error = 0.0;

            for (int i = 0; i < input.Count; i++)
            {
                if (X[i].ConstraintStatus)
                    continue;

                SparseElement m = input.M[i];

                double[] bufValue = m.Value;
                int[] bufIndex = m.Index;

                double bValue = (1.0 / input.D[i]) * X[i].X;

                for (int j = 0; j < m.Count; j++)
                    bValue += bufValue[j] * X[bufIndex[j]].X;

                error += (bValue - input.B[i]) * (bValue - input.B[i]);
            }

            return error;
        }
    }
}
