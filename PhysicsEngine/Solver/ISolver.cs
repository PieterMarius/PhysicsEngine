
namespace SharpPhysicsEngine.LCPSolver
{
	internal interface ISolver
	{
		SolutionValues[] Solve(
            LinearProblemProperties linearProblemProperties,
            SolutionValues[] X = null);
        		
		SolverParameters GetSolverParameters();
	}
}

