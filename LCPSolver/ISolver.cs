using System;

namespace LCPSolver
{
	public interface ISolver
	{
		double[] Solve(LinearProblemProperties linearProblemProperties);

		double[] GetError(
			LinearProblemProperties linearProblemProperties, 
			double[] X);

		double GetMediumSquareError (
			LinearProblemProperties input,
			double[] X);
	}
}

