using System;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.LCPSolver
{
	public static class ClampSolution
	{
		public static SolutionValues Clamp(
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

                    double frictionLimit = X[input.Constraints[i].Value].X * input.ConstraintLimit[i];

                    if (X[i].X < -frictionLimit)
                        return new SolutionValues(-frictionLimit, true);
                    if (X[i].X > frictionLimit)
                        return new SolutionValues(frictionLimit, true);

                    return new SolutionValues(X[i].X, false);

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
                
        public static double Clamp(
            LinearProblemProperties input,
            double[] X,
            int i)
        {
            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    if (X[i] < 0)
                        return 0.0;
                    else
                        return X[i];

                case ConstraintType.Friction:
                   
                    double frictionLimit = X[input.Constraints[i].Value] * input.ConstraintLimit[i];

                    if (X[i] < -frictionLimit)
                        return -frictionLimit;
                    if (X[i] > frictionLimit)
                        return frictionLimit;

                    return X[i];
                    

                case ConstraintType.JointMotor:
                    double limit = input.ConstraintLimit[i];

                    if (X[i] < -limit)
                        return -limit;
                    if (X[i] > limit)
                        return limit;

                    return X[i];

                default:
                    return X[i];
            }
        }

        public static bool GetIfClamped(
            LinearProblemProperties input,
            double[] X,
            int i)
        {

            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    if (Math.Abs(X[i]) < 1E-50)
                        return true;
                    else
                        return false;

                case ConstraintType.Friction:
                    
                    double frictionLimit = X[input.Constraints[i].Value] * input.ConstraintLimit[i];

                    if (Math.Abs(X[i] + frictionLimit) < 1E-50 ||
                        Math.Abs(X[i] - frictionLimit) < 1E-50)
                        return true;
                    else
                        return false;
                    
                case ConstraintType.JointMotor:
                    if (Math.Abs(X[i] - input.ConstraintLimit[i]) < 1E-50 ||
                        Math.Abs(X[i] + input.ConstraintLimit[i]) < 1E-50)
                        return true;
                    else
                        return false;
                                        
                default:
                    return false;
            }
        }

        public static void GetConstraintValues(
            LinearProblemProperties input,
            double[] x,
            int i,
            ref double Lower,
            ref double Upper)
        {
            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    Lower = 0.0;
                    Upper = double.MaxValue;
                    break;   

                case ConstraintType.Friction:

                    double frictionLimit = x[input.Constraints[i].Value] * input.ConstraintLimit[i];

                    Lower = -frictionLimit;
                    Upper = frictionLimit;
                    break;

                case ConstraintType.JointMotor:
                    double limit = input.ConstraintLimit[i];

                    Lower = -input.ConstraintLimit[i];
                    Upper = input.ConstraintLimit[i];
                    break;

                default:
                    Lower = double.MinValue;
                    Upper = double.MaxValue;
                    break;
            }
        }
    }
}

