
namespace LCPSolver
{
	public interface ISolver
	{
		SolutionValues[] Solve(
            LinearProblemProperties linearProblemProperties,
            SolutionValues[] X = null);
        		
		SolverParameters GetSolverParameters();
	}
}

