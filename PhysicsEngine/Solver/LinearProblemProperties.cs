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
using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;
using System.Linq;
using System;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class LinearProblemProperties : IEqualityComparer<LinearProblemProperties>
    {
        #region Properties
                
        //Matrix A (N*N)
        public readonly SparseElement[] M;

        //Vector B (N)
        public readonly double[] B;

        //Inverse Diagonal of matrix A (N)
        public readonly double[] InvD;

        //Diagonal of matrix A(N)
        public readonly double[] D;

        //Constraint
        public readonly double[] ConstraintLimit;

        //Contraint type
        public ConstraintType[] ConstraintType { get; private set; }

        //Constraint joint
        public readonly int?[] Constraints;

        //Size of the system
        public readonly int Count;

        #endregion

        #region Constructor

        public LinearProblemProperties(
            SparseElement[] M,
            double[] B,
            double[] D,
            double[] InvD,
            double[] constraintLimit,
            ConstraintType[] constraintType,
            int?[] constraints)
        {
            this.M = M;
            this.B = B;
            this.D = D;
            this.InvD = InvD;
            ConstraintLimit = constraintLimit;
            ConstraintType = constraintType;
            Constraints = constraints;
            Count = B.Length;
        }

        #endregion

        #region Public Methods

        public double[][] GetOriginalMatrix()
        {
            double[][] matrix = new double[Count][];

            for (int i = 0; i < Count; i++)
            {
                matrix[i] = GetOriginalRow(i);
                matrix[i][i] = D[i];
            }

            return matrix;
        }

        public List<List<double>> GetOriginalMatrixList()
        {
            List<List<double>> matrix = new List<List<double>>();

            for (int i = 0; i < Count; i++)
            {
                matrix.Add(GetOriginalRow(i).ToList());
                matrix[i][i] = D[i];
            }

            return matrix;
        }

        public SparseElement[] GetOriginalSparseMatrix()
        {
            SparseElement[] originalMatrix = new SparseElement[M.Length];

            for (int i = 0; i < M.Length; i++)
            {
                List<double> valueList = M[i].Value.ToList();
                List<int> indexList = M[i].Index.ToList();

                valueList.Add(D[i]);
                indexList.Add(i);

                originalMatrix[i] = new SparseElement(valueList.ToArray(), indexList.ToArray());
            }

            return originalMatrix;
        }

        public void SetConstraintType(ConstraintType type, int index)
        {
            ConstraintType[index] = type;
        }

        public double ComputeSolverError(
            double[] X)
        {
            double[][] matrix = GetOriginalMatrix();

            double error = 0.0;

            for (int i = 0; i < Count; i++)
            {
                double diffError = CalculateRowError(matrix[i], X, B[i]);

                error += diffError * diffError;
            }
      
            return error;
        }

        public bool Equals(LinearProblemProperties x, LinearProblemProperties y)
        {
            if (x.M.Length != y.M.Length)
                return false;

            if (x.B.Length != y.B.Length)
                return false;

            if (x.InvD.Length != y.InvD.Length)
                return false;

            for (int i = 0; i < x.B.Length; i++)
            {
                if (!x.B[i].Equals(y.B[i]))
                    return false;
            }

            for (int i = 0; i < x.InvD.Length; i++)
            {
                if (!x.InvD[i].Equals(y.InvD[i]))
                    return false;
            }

            double[][] A = x.GetOriginalMatrix();
            double[][] B = y.GetOriginalMatrix();
            
            for (int i = 0; i < A.Length; i++)
            {
                for (int j = 0; j < A[i].Length; j++)
                {
                    if (Math.Abs(A[i][j] - B[i][j]) > 1E-12)
                        return false;
                }
            }

            return true;
        }

        public int GetHashCode(LinearProblemProperties obj)
        {
            return this.GetHashCode();
        }

        #endregion

        #region Private Methods

        private double[] GetOriginalRow(int index)
        {
            double[] row = new double[Count];

            SparseElement element = M[index];

            for (int i = 0; i < element.Count; i++)
                row[element.Index[i]] = element.Value[i];

            return row;
        }

        private double CalculateRowError(
            double[] row,
            double[] X,
            double expected)
        {
            double bValue = 0.0;
            for (int j = 0; j < Count; j++)
                bValue += row[j] * X[j];

            return bValue - expected;
        }

        #endregion

    }
}

