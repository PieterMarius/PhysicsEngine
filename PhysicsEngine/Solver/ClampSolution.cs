﻿/******************************************************************************
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

