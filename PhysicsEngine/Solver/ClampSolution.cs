using System;
using System.Linq;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.LCPSolver
{
    internal static class ClampSolution
    {
        public static double Clamp(
            LinearProblemProperties input,
            double[] X,
            int i)
        {

            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    return (X[i] < 0) ?
                        0.0:
                        X[i];

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
                    return true;

                case ConstraintType.Friction:
                    return true;
                
                case ConstraintType.JointMotor:
                    return true;
               
                default:
                    return false;
            }
        }
    }
}

