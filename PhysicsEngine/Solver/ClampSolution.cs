using System;
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
                //if (Math.Abs(X[i]) < 1E-50)
                //    return true;
                //else
                //    return false;

                case ConstraintType.Friction:
                    return true;
                //double frictionLimit = X[input.Constraints[i].Value] * input.ConstraintLimit[i];

                //if (X[i] < -frictionLimit)
                //    return true;
                //if (X[i] > frictionLimit)
                //    return true;

                //return false;

                case ConstraintType.JointMotor:
                    return true;
                //double limit = input.ConstraintLimit[i];

                //if (X[i] < -limit)
                //    return true;
                //if (X[i] > limit)
                //    return true;

                //return false;

                default:
                    return false;
            }
        }

        //public static bool GetIfClamped(
        //    ConstraintType constraintType)
        //{

        //    switch (constraintType)
        //    {
        //        case ConstraintType.Collision:
        //        case ConstraintType.JointLimit:
        //            return true;

        //        case ConstraintType.Friction:
        //            return true;

        //        case ConstraintType.JointMotor:
        //            return true;

        //        default:
        //            return false;
        //    }
        //}
    }
}

