using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace LCPSolver
{
	public static class ClampSolution
	{
		public static SolutionValues ClampX(
			LinearProblemProperties input,
			SolutionValues[] X,
            	int i)
		{

            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    if (X[i].X < 0)
                        return new SolutionValues(0.0, true);
                    else
                        return new SolutionValues(X[i].X, false);
                                        
				case ConstraintType.Friction:
                    if (Math.Abs(X[i].X) < 1E-50)
                        return new SolutionValues(0.0, true);
                    else
                    { 
                        double frictionLimit = X[input.Constraints[i][0].Value].X * input.ConstraintLimit[i];

                        if (X[i].X < -frictionLimit)
                            return new SolutionValues(-frictionLimit, true);
                        if (X[i].X > frictionLimit)
                            return new SolutionValues(frictionLimit, true);

                        return new SolutionValues(X[i].X, false);
                    }

                case ConstraintType.JointMotor:
					double limit = input.ConstraintLimit[i];

                    if (X[i].X < -limit)
                        return new SolutionValues(-limit, true);
                    if (X[i].X > limit)
                        return new SolutionValues(limit, true);

                    return new SolutionValues(X[i].X, false);
                    				
				default:
                    return new SolutionValues(X[i].X, false);
            }
		}
	}
}

