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

using SharpEngineMathUtility;
using static SharpEngineMathUtility.SparseMatrix;
using static SharpEngineMathUtility.MathUtils;
using System.Linq;
using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;
using static SharpPhysicsEngine.LCPSolver.RedBlackProjectedGaussSeidel;
using System;
using SharpEngineMathUtility.Solver;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class ProjectedConjugateGradient : ISolver
    {
        #region Fields

        RedBlackProjectedGaussSeidel gaussSeidelSolver;
        public readonly SolverParameters SolverParameters;

        #endregion

        #region Constructor

        public ProjectedConjugateGradient(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;

            var gaussSeidelSolverParam = new SolverParameters(
                                                          1,
                                                          SolverParameters.ErrorTolerance,
                                                          1.0,
                                                          SolverParameters.MaxThreadNumber);

            gaussSeidelSolver = new RedBlackProjectedGaussSeidel(gaussSeidelSolverParam);
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
        }

        public double[] Solve(
            LinearProblemProperties linearProblemProperties,
            double[] x)
        {
            return null;
        }

        public double[] Solve(
            LinearProblemProperties globalLP,
            LinearProblemProperties contactLP,
            LinearProblemProperties jointLP,
            LinearProblemProperties jointWhitLimitLP,
            double[] startValues)
        {
            if (startValues == null)
                startValues = new double[globalLP.Count];

            double[] x = startValues;
            double[] xOld = new double[globalLP.Count];
            double[] xJoint = new double[(jointLP != null) ? jointLP.Count : 0];
            double[] xContact = new double[(contactLP != null) ? contactLP.Count : 0];
            double[] xJointWithLimit = new double[(jointWhitLimitLP != null) ? jointWhitLimitLP.Count : 0];

            var CGSolver = new ConjugateGradient();

            var gaussSeidelSolverParam = new SolverParameters(
                                                          1,
                                                          SolverParameters.ErrorTolerance,
                                                          1.0,
                                                          SolverParameters.MaxThreadNumber);

            var PGS = new ProjectedGaussSeidel(gaussSeidelSolverParam);

            SparseMatrix A = new SparseMatrix();

            if (jointLP != null)
            {
                A = jointLP.GetOriginalSparseMatrix();
            }
            
            for (int i = 0; i < 50; i++)
            {
                //Solve Joint 
                if (jointLP != null)
                    xJoint = CGSolver.Solve(A, jointLP.B, xJoint, 10);

                //Solve Contact
                if(contactLP != null)
                    xContact = PGS.Solve(contactLP, xContact);

                //Solve Joint With Limit
                if(jointWhitLimitLP != null)
                    xJointWithLimit = PGS.Solve(jointWhitLimitLP, xJointWithLimit);

                //Set Values
                x = SetGlobalValues(globalLP.ConstraintType, x, xJoint, xContact, xJointWithLimit);

                //Solve Global LP
                x = PGS.Solve(globalLP, x);

                //SetValues
                SetSubsystemValues(globalLP.ConstraintType, x, ref xJoint, ref xContact, ref xJointWithLimit);

                double actualSolverError = SolverHelper.ComputeSolverError(x, xOld);

                if (actualSolverError < SolverParameters.ErrorTolerance)
                    return x;

                Array.Copy(x, xOld, x.Length);
            }
                       

            return x;
        }

        #endregion

        #region Private Methods
                
        private double[] SetGlobalValues(
            ConstraintType[] constraintsType,
            double[] x,
            double[] xJoint,
            double[] xContact,
            double[] xJointWithLimit)
        {
            int ij = 0, ic = 0, ijl = 0;

            for (int i = 0; i < constraintsType.Length; i++)
            {
                if (constraintsType[i] == ConstraintType.Joint ||
                    constraintsType[i] == ConstraintType.SoftJoint)
                {
                    x[i] = xJoint[ij];
                    ij++;
                    continue;
                }

                if (constraintsType[i] == ConstraintType.Collision ||
                    constraintsType[i] == ConstraintType.Friction)
                {
                    x[i] = xContact[ic];
                    ic++;
                    continue;
                }

                if (constraintsType[i] == ConstraintType.JointLimit ||
                    constraintsType[i] == ConstraintType.JointMotor)
                {
                    x[i] = xJointWithLimit[ijl];
                    ijl++;
                    continue;
                }
            }
            return x;
        }

        private void SetSubsystemValues(
            ConstraintType[] constraintsType,
            double[] x,
            ref double[] xJoint,
            ref double[] xContact,
            ref double[] xJointWithLimit)
        {
            int ij = 0, ic = 0, ijl = 0;

            for (int i = 0; i < constraintsType.Length; i++)
            {
                if (constraintsType[i] == ConstraintType.Joint ||
                    constraintsType[i] == ConstraintType.SoftJoint)
                {
                    xJoint[ij] = x[i];
                    ij++;
                    continue;
                }

                if (constraintsType[i] == ConstraintType.Collision ||
                    constraintsType[i] == ConstraintType.Friction)
                {
                    xContact[ic] = xContact[i];
                    ic++;
                    continue;
                }

                if (constraintsType[i] == ConstraintType.JointLimit ||
                    constraintsType[i] == ConstraintType.JointMotor)
                {
                    xJointWithLimit[ijl] = x[i];
                    ijl++;
                    continue;
                }
            }
        }

        public static double ComputeSolverError(
            double[] x,
            double[] oldx)
        {
            double[] actualSolverErrorDiff = MathUtils.Minus(x, oldx);
            return MathUtils.Dot(actualSolverErrorDiff, actualSolverErrorDiff);
        }

        #endregion
    }
}
