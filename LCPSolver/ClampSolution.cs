using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace LCPSolver
{
	public static class ClampSolution
	{
		public static double ClampX(
			LinearProblemProperties input,
			double[] X,
			int i)
		{

			switch (input.ConstraintType[i]) 
			{
				case ConstraintType.JointLimit:
					return Math.Max (0.0, X [i]);

				case ConstraintType.Collision:
					return Math.Max (0.0, X [i]);

				case ConstraintType.Friction:

					double frictionLimit = X [i + input.Constraints[i].Value] * input.ConstraintLimit[i];

					return GeometryUtilities.Clamp(
						X[i],
						frictionLimit,
						-frictionLimit);

				case ConstraintType.JointMotor:
					
					double limit = input.ConstraintLimit[i];

					return GeometryUtilities.Clamp (
						X [i],
						limit,
						-limit);
				
				default:
					return X [i];
			}
		}
	}
}

