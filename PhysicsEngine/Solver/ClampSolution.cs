using System;
using ShapeDefinition;

namespace LCPSolver
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
                    
                    double frictionLimit = X[input.Constraints[i][0].Value].X * input.ConstraintLimit[i];

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
                   
                    double frictionLimit = X[input.Constraints[i][0].Value] * input.ConstraintLimit[i];

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
                    
                    double frictionLimit = X[input.Constraints[i][0].Value] * input.ConstraintLimit[i];

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
            ref double Min,
            ref double Max)
        {
            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    Min = 0.0;
                    Max = double.MaxValue;
                    break;   

                case ConstraintType.Friction:

                    double frictionLimit = x[input.Constraints[i][0].Value] * input.ConstraintLimit[i];

                    Min = -frictionLimit;
                    Max = frictionLimit;
                    break;

                case ConstraintType.JointMotor:
                    double limit = input.ConstraintLimit[i];

                    Min = -input.ConstraintLimit[i];
                    Max = input.ConstraintLimit[i];
                    break;

                default:
                    Min = double.MinValue;
                    Max = double.MaxValue;
                    break;
            }
        }
    }
}

