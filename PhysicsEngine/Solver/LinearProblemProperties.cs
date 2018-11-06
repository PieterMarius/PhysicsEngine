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
using SharpPhysicsEngine.Helper;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class LinearProblemProperties : IEqualityComparer<LinearProblemProperties>
    {
        #region Properties
                
        //Matrix A (N*N)
        public readonly SparseMatrix M;

        //Vector B (N)
        public readonly double[] B;

        //Inverse Diagonal of matrix A (N)
        public readonly double[] InvD;

        //Diagonal of matrix A(N)
        public readonly double[] D;

        //Start Values
        public readonly double[] StartImpulse;

        //Constraint
        public readonly double[] ConstraintLimit;

        //Contraint type
        public ConstraintType[] ConstraintType { get; private set; }

        //Constraint joint
        public readonly int?[] Constraints;

        //Constraints connection graph
        public readonly Graph ConstrGraph;

        //Size of the system
        public readonly int Count;

        //Friction Direction 
        public readonly int FrictionDirections;

        #endregion

        #region Constructor

        public LinearProblemProperties(
            SparseMatrix M,
            double[] B,
            double[] D,
            double[] InvD,
            double[] constraintLimit,
            ConstraintType[] constraintType,
            Graph constrGraph,
            int?[] constraints,
            double[] startImpulse)
        {
            this.M = M;
            this.B = B;
            this.D = D;
            this.InvD = InvD;
            ConstraintLimit = constraintLimit;
            ConstraintType = constraintType;
            Constraints = constraints;
            ConstrGraph = constrGraph;
            Count = B.Length;
            StartImpulse = startImpulse;
        }

        public LinearProblemProperties(
            LinearProblemBaseProperties baseProperties,
            Graph constrGraph)
        {
            M = baseProperties.M;
            B = baseProperties.B;
            D = baseProperties.D;
            InvD = baseProperties.InvD;
            ConstraintLimit = baseProperties.ConstraintLimit;
            ConstraintType = baseProperties.ConstraintType;
            Constraints = baseProperties.ConstraintsArray;
            ConstrGraph = constrGraph;
            Count = B.Length;
            StartImpulse = baseProperties.StartValue;
        }

        //public LinearProblemProperties(
        //    LinearProblemBaseProperties[] baseProperties,
        //    Graph constrGraph)
        //{
        //    this.M = Array.ConvertAll(baseProperties, x => x.M);
        //    this.B = Array.ConvertAll(baseProperties, x => x.B);
        //    this.D = Array.ConvertAll(baseProperties, x => x.D);
        //    this.InvD = Array.ConvertAll(baseProperties, x => x.InvD);
        //    ConstraintLimit = Array.ConvertAll(baseProperties, x => x.ConstraintLimit);
        //    ConstraintType = Array.ConvertAll(baseProperties, x => x.ConstraintType);
        //    Constraints = Array.ConvertAll(baseProperties, x => x.ConstraintsArray);
        //    ConstrGraph = constrGraph;
        //    Count = B.Length;
        //}

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

        public SparseMatrix GetOriginalSparseMatrix()
        {
            SparseMatrix originalMatrix = new SparseMatrix(M.n, M.m);

            for (int i = 0; i < M.n; i++)
            {
                List<double> valueList = M.Rows[i].Value.ToList();
                List<int> indexList = M.Rows[i].Index.ToList();

                valueList.Add(D[i]);
                indexList.Add(i);

                originalMatrix.Rows[i] = new SparseVector(valueList.ToArray(), indexList.ToArray(), M.m);
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
            if (x.M.n != y.M.n)
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
            return GetHashCode();
        }

        #endregion

        #region Private Methods

        private double[] GetOriginalRow(int index)
        {
            double[] row = new double[Count];

            SparseVector element = M.Rows[index];

            for (int i = 0; i < element.Index.Length; i++)
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

