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

using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Helper
{
    internal struct LinearProblemBaseProperties
    {
        #region Fields

        public double[] B;
        public double[] D;
        public double[] InvD;
        public ConstraintType[] ConstraintType;
        public double[] ConstraintLimit;
        public SparseElement[] M;
        public int?[] ConstraintsArray;
        public double[] StartValue;

        #endregion

        #region Constructor

        public LinearProblemBaseProperties(int constraintLength)
        {
            B = new double[constraintLength];
            D = new double[constraintLength];
            InvD = new double[constraintLength];
            ConstraintType = new ConstraintType[constraintLength];
            ConstraintLimit = new double[constraintLength];
            M = new SparseElement[constraintLength];
            ConstraintsArray = new int?[constraintLength];
            StartValue = new double[constraintLength];
        }

        #endregion
    }
}
