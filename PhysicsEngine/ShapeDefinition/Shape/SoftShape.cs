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
using SharpEngineMathUtility;
using System.Collections.Generic;
using System.Linq;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpPhysicsEngine.CollisionEngine.Dynamic_Bounding_Tree;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class SimSoftShape : Shape, ISoftShape, IAABB
    {
     
        #region Simulation Properties
                
        public SoftShapePoint[] ShapePoints { get; private set; }

        public TriangleMesh[] Triangle { get; private set; }

        public SoftPoint[] Sphere { get; private set; }

        public List<SoftConstraint> SoftConstraint { get; private set; }

        public IShapeConvexDecomposition ConvexDecomposition { get; private set; }

        public double DecompositionParameter { get; private set; }

        public double AngularErrorReductionParam { get; private set; }

        public double AngularSpringCoeff { get; private set; }

        public override Vector3d[] Vertices { get { return null; } }

        #endregion

        #region Private Const

        private const double diameter = 0.01;

        #endregion

        #region Constructor

        public SimSoftShape(
            TriangleMesh[] triangleIndex,
            Vector3d[] shapePoint,
            ConstraintIndex[] softConstraints,
            double mass,
            double decompositionParam,
            double restoreCoeff,
            double springCoeff,
            double angularErrorReductionParam,
            double angularSpringCoeff) : base()
        {
            ObjectType = ObjectType.SoftBody;
            Triangle = triangleIndex;
            MassInfo.InverseInertiaTensor = Matrix3x3.IdentityMatrix();
            MassInfo.Mass = mass;
            AddSoftShapePoint(shapePoint);
            BuildSoftConstraints(softConstraints, restoreCoeff, springCoeff, angularErrorReductionParam, angularSpringCoeff);
            SleepingFrameCount = 0;
        }

        public SimSoftShape(
            TriangleMesh[] triangleIndex,
            Vector3d[] shapePoint,
            ConstraintIndex[] softConstraints,
            double mass,
            double decompositionParam,
            double restoreCoeff,
            double springCoeff) :
            this(triangleIndex, shapePoint, softConstraints, mass, decompositionParam, restoreCoeff, springCoeff, 0.0, 0.0)
        { }

        public SimSoftShape(
            TriangleMesh[] triangleIndex,
            Vector3d[] shapePoint,
            Vector3d startPosition,
            double mass,
            double decompositionParam,
            double errorReductionParam,
            double springCoeff,
            double angularErrorReductionParam,
            double angularSpringCoeff) : base()
        {
            Triangle = triangleIndex;

            DecompositionParameter = decompositionParam;
            MassInfo.Mass = mass;
            MassInfo.InverseInertiaTensor = Matrix3x3.IdentityMatrix();

            Position = startPosition;

            AddSoftShapePoint(shapePoint);
            BuildSoftConstraints(errorReductionParam, springCoeff, angularErrorReductionParam, angularSpringCoeff);
                        
            SleepingFrameCount = 0;

            SetAABB();
        }

        public SimSoftShape(
            TriangleMesh[] triangleIndex,
            Vector3d[] shapePoint,
            Vector3d startPosition,
            double mass,
            double decompositionParam,
            double errorReductionParam,
            double springCoeff)
            : this(triangleIndex, shapePoint, startPosition, mass, decompositionParam, errorReductionParam, springCoeff, 0.0, 0.0)
        { }

        #endregion

        #region Public Methods

        public override void SetMass(double mass)
        {
            MassInfo.Mass = mass;
        }
               
        public override void SetAABB()
        {
            AABBox = AABB.GetShapePointAABB(ShapePoints, null);
        }

        public AABB GetAABB()
        {
            return AABBox;
        }

        public void SetPointsMass(double mass)
        {
            MassInfo.Mass = mass;
            
            if (IsStatic)
            {
                MassInfo.Mass = 0.0;
                MassInfo.InverseMass = 0.0;
            }
            else if (MassInfo.Mass > 0.0)
                MassInfo.InverseMass = 1.0 / MassInfo.Mass;
        }

        public void SetShapePoint(SoftShapePoint[] shapePoint)
        {
            ShapePoints = shapePoint;
        }
        
        public void SetConstraintsErrorReductionParam(double restoreCoeff)
        {
            foreach (var item in SoftConstraint)
                item.SetErrorReductionParam(restoreCoeff);
        }

        public void SetConstraintsSpringCoefficient(double springCoeff)
        {
            foreach (var item in SoftConstraint)
                item.SetSpringCoefficient(springCoeff);
        }

        public void AddConstraint(SoftConstraint constraint)
        {
            SoftConstraint.Add(constraint);
        }

        public void RemoveConstraint(int index)
        {
            SoftConstraint.RemoveAt(index);
        }

        public void SetDecompositionParameter(double decompositionParam)
        {
            DecompositionParameter = decompositionParam;
        }

        public override void Rotate(Vector3d versor, double angle)
        {
            throw new NotImplementedException();
        }

        public void AddToConstraintsErrorReductionParam(double value)
        {
            foreach (var item in SoftConstraint)
            {
                double val = item.GetErrorReductionParam() + value;
                    if (val > 0.0)
                item.SetErrorReductionParam(val);
            }
        }

        public void AddToConstraintsSpringCoefficient(double value)
        {
            foreach (var item in SoftConstraint)
            {
                double val = item.GetSpringCoefficient() + value;
                if (val > 0.0)
                    item.SetSpringCoefficient(val);
            }
        }

        public int CompareTo(object obj)
        {
            return obj.GetHashCode().CompareTo(GetHashCode());
        }

        #endregion

        #region Private Methods

        private void AddSoftShapePoint(
            Vector3d[] points)
        {
            ShapePoints = new SoftShapePoint[points.Length];

            double mass = MassInfo.Mass * (1.0 / points.Length);
            double inverseMass = 1.0 / mass;

            Matrix3x3 inertiaTensor = Matrix3x3.IdentityMatrix() *
                                      (diameter * diameter * 0.4);

            for (int i = 0; i < points.Length; i++)
            {
                ShapePoints[i] = new SoftShapePoint(diameter);
                ShapePoints[i].SetStartPosition(points[i]);
                ShapePoints[i].SetPosition(points[i] + Position);

                Vector3d r = -1.0 * ShapePoints[i].StartPosition;
                var baseInertiaTensor = inertiaTensor + (Matrix3x3.IdentityMatrix() * r.Dot(r) - Matrix3x3.OuterProduct(r, r)) * mass;

                Matrix3x3 baseTensors = Matrix3x3.Invert(baseInertiaTensor);
                
                ShapePoints[i].SetLinearVelocity(LinearVelocity);
                ShapePoints[i].SetAngularVelocity(AngularVelocity);
                ShapePoints[i].SetMass(mass);
                ShapePoints[i].SetInverseMass(inverseMass);
                ShapePoints[i].SetInverseBaseInertiaTensor(baseTensors);
                ShapePoints[i].SetInverseInertiaTensor(baseTensors);
                ShapePoints[i].SetRotationStatus(new Quaternion(1.0, 0.0, 0.0, 0.0));
                ShapePoints[i].SetRotationMatrix(Quaternion.ConvertToMatrix(RotationStatus));
            }
        }
                
        private void BuildSoftConstraints(
            ConstraintIndex[] constraintIndex,
            double restoreCoeff,
            double springCoeff,
            double angularErrorReductionParam,
            double angularSpringCoeff)
        {
            SoftConstraint = new List<SoftConstraint>();
            HashSet<ConstraintIndex> indexHashSet = new HashSet<ConstraintIndex>();

            for (int i = 0; i < constraintIndex.Length; i++)
            {
                if (indexHashSet.Add(constraintIndex[i]))
                {
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[constraintIndex[i].IndexA],
                        ShapePoints[constraintIndex[i].IndexB],
                        this,
                        restoreCoeff,
                        springCoeff,
                        angularErrorReductionParam,
                        angularSpringCoeff));
                }
            }
        }

        private void BuildSoftConstraints(
            double restoreCoeff,
            double springCoeff,
            double angularErrReductionParam,
            double angularSpringCoeff)
        {
            SoftConstraint = new List<SoftConstraint>();
            HashSet<ConstraintIndex> indexHashSet = new HashSet<ConstraintIndex>();
            
            foreach (var triangle in Triangle.Select((value, index) => new { value, index }))
            {
                if(indexHashSet.Add(new ConstraintIndex(triangle.value.a, triangle.value.b)) &&
                   triangle.value.a != triangle.value.b)
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[triangle.value.a],
                        ShapePoints[triangle.value.b],
                        this,
                        restoreCoeff,
                        springCoeff,
                        angularErrReductionParam,
                        angularSpringCoeff));

                if (indexHashSet.Add(new ConstraintIndex(triangle.value.a, triangle.value.c)) &&
                    triangle.value.a != triangle.value.c)
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[triangle.value.a],
                        ShapePoints[triangle.value.c],
                        this,
                        restoreCoeff,
                        springCoeff,
                        angularErrReductionParam,
                        angularSpringCoeff));

                if (indexHashSet.Add(new ConstraintIndex(triangle.value.b, triangle.value.c)) &&
                    triangle.value.b != triangle.value.c)
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[triangle.value.b],
                        ShapePoints[triangle.value.c],
                        this,
                        restoreCoeff,
                        springCoeff,
                        angularErrReductionParam,
                        angularSpringCoeff));

                //Add triangle index to shape points
                ShapePoints[triangle.value.a].AddTrianglesIndex(triangle.index);
                ShapePoints[triangle.value.b].AddTrianglesIndex(triangle.index);
                ShapePoints[triangle.value.c].AddTrianglesIndex(triangle.index);
            }
        }

        
        #endregion
    }
}
