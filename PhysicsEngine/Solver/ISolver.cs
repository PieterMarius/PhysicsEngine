
namespace SharpPhysicsEngine.LCPSolver
{
	internal interface ISolver
	{
		double[] Solve(
            LinearProblemProperties linearProblemProperties,
            double[] x = null);
        		
		SolverParameters GetSolverParameters();
	}
}

