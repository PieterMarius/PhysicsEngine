/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.LCPSolver
{
    internal static class ClampSolution
    {
        public static double Clamp(
            LinearProblemProperties input,
            double xValue,
            ref double[] X,
            int i)
        {

            switch (input.ConstraintType[i])
            {
                case ConstraintType.Collision:
                case ConstraintType.JointLimit:
                    return (xValue < 0.0) ?
                        0.0:
                        xValue;

                case ConstraintType.Friction:

                    //Isotropic friction -> sqrt(c1^2+c2^2) <= fn*U

                    int sign = (input.Constraints[i].Value < 0) ? -1 : 1;
                    
                    int normalIndex = sign * input.Constraints[i].Value;
                    double frictionLimit = X[normalIndex] * input.ConstraintLimit[i];
                    int idx1 = normalIndex + sign;
                    int idx2 = normalIndex + sign * 2;

                    if (frictionLimit == 0.0)
                    {
                        X[idx1] = 0.0;
                        X[idx2] = 0.0;

                        return 0.0;
                    }

                    double directionA = X[idx1];
                    double directionB = X[idx2];
                    double frictionValue = Math.Sqrt(directionA * directionA + directionB * directionB);

                    if (frictionValue > frictionLimit)
                    {
                        Vector2d frictionNormal = new Vector2d(directionA / frictionValue, directionB / frictionValue);

                        X[idx1] = frictionNormal.x * frictionLimit;
                        X[idx2] = frictionNormal.y * frictionLimit;

                        return (Math.Abs(i - normalIndex) == 1) ?
                                X[idx1] :
                                X[idx2];
                    }

                    return xValue;

                case ConstraintType.JointMotor:
                    double limit = input.ConstraintLimit[i];

                    if (xValue < -limit)
                        return -limit;
                    if (xValue > limit)
                        return limit;

                    return xValue;

                default:
                    return xValue;
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

