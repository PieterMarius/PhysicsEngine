using System;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.Solver;

namespace SharpPhysicsEngine.LCPSolver
{
    internal static class ClampSolution
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

        public static ClampProperties Clamp(
            LinearProblemProperties input,
            double[] p,
            double[] X,
            int i)
        {
            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    if (X[i] < 0.0)
                        return new ClampProperties(0.0, FrictionStatus.None);
                    else
                        return new ClampProperties(X[i], FrictionStatus.None);

                case ConstraintType.Friction:

                    if (X[input.Constraints[i].Value] < 0.0)
                        return new ClampProperties(0.0, FrictionStatus.Gap);
                    
                    double frictionLimit = X[input.Constraints[i].Value] * input.ConstraintLimit[i];
                    
                    if (X[i] < -frictionLimit)
                        return new ClampProperties(- frictionLimit, FrictionStatus.SlipNegative);

                    if (X[i] > frictionLimit)
                        return new ClampProperties(frictionLimit, FrictionStatus.SlipPositive);

                    if (Math.Abs(p[i]) < 1E-50 && X[i] < 0.0)
                        return new ClampProperties(- frictionLimit, FrictionStatus.SlipNegative);

                    if (Math.Abs(p[i]) < 1E-50 && X[i] > 0.0)
                        return new ClampProperties(frictionLimit, FrictionStatus.SlipPositive);


                    return new ClampProperties(X[i], FrictionStatus.Stick);


                case ConstraintType.JointMotor:
                    double limit = input.ConstraintLimit[i];

                    if (X[i] < -limit)
                        return new ClampProperties(-limit, FrictionStatus.None);
                    if (X[i] > limit)
                        return new ClampProperties(limit, FrictionStatus.None);

                    return new ClampProperties(X[i], FrictionStatus.None);

                default:
                    return new ClampProperties(X[i], FrictionStatus.None);
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

                    if (Math.Abs(X[i]) - frictionLimit < 1E-50 )
                        return true;
                    else
                        return false;

                case ConstraintType.JointMotor:
                    if (Math.Abs(X[i]) - input.ConstraintLimit[i] < 1E-50)
                        return true;
                    else
                        return false;

                default:
                    return false;
            }
        }

        public static void GetConstraintValues(
            LinearProblemProperties input,
            ClampProperties[] x,
            int i,
            ref double? Lower,
            ref double? Upper)
        {
            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    Lower = 0.0;
                    Upper = null;
                    break;

                case ConstraintType.Friction:

                    double frictionLimit = x[input.Constraints[i].Value].Value * input.ConstraintLimit[i];

                    Lower = -frictionLimit;
                    Upper = frictionLimit;
                    break;

                case ConstraintType.JointMotor:
                    double limit = input.ConstraintLimit[i];

                    Lower = -input.ConstraintLimit[i];
                    Upper = input.ConstraintLimit[i];
                    break;

                default:
                    Lower = null;
                    Upper = null;
                    break;
            }
        }
    }
}

