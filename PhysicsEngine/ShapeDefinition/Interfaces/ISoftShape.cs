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

using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using System.Collections.Generic;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal interface ISoftShape
    {
        SoftShapePoint[] ShapePoints { get; }
        TriangleMesh[] Triangle { get; }
        SoftPoint[] Sphere { get; }
        List<SoftConstraint> SoftConstraint { get; }
        double DecompositionParameter { get; }
        double AngularErrorReductionParam { get; }
        double AngularSpringCoeff { get; }

        
        void SetPointsMass(double mass);
        void SetShapePoint(SoftShapePoint[] shapePoint);
        void SetAABB();
        void AddConstraint(SoftConstraint constraint);
        void RemoveConstraint(int index);
        void SetDecompositionParameter(double decompositionParam);
        void SetConstraintsErrorReductionParam(double errorReduction);
        void SetConstraintsSpringCoefficient(double springCoeff);
        void AddToConstraintsErrorReductionParam(double value);
        void AddToConstraintsSpringCoefficient(double value);
        void SetID(int id);
    }
}
