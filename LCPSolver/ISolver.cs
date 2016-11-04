
namespace LCPSolver
{
	public interface ISolver
	{
		double[] Solve(LinearProblemProperties linearProblemProperties);

		double GetDifferentialMSE();
		SolverParameters GetSolverParameters();
	}
}

