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
using System.Collections.Generic;
using System.Linq;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class SoftShape : Shape, ISoftShape
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

        #endregion

        #region Private Const

        private const double diameter = 0.01;

        #endregion

        #region Constructor

        public SoftShape(
            TriangleMesh[] triangleIndex,
            Vector3[] shapePoint,
            ConstraintIndex[] softConstraints,
            double mass,
            double decompositionParam,
            double restoreCoeff,
            double springCoeff,
            double angularErrorReductionParam,
            double angularSpringCoeff)
        {
            ObjectType = ObjectType.SoftBody;
            Triangle = triangleIndex;
            InertiaTensor = Matrix3x3.IdentityMatrix();
            Mass = mass;
            AddSoftShapePoint(shapePoint);
            BuildSoftConstraints(softConstraints, restoreCoeff, springCoeff, angularErrorReductionParam, angularSpringCoeff);
            SleepingFrameCount = 0;
        }

        public SoftShape(
            TriangleMesh[] triangleIndex,
            Vector3[] shapePoint,
            ConstraintIndex[] softConstraints,
            double mass,
            double decompositionParam,
            double restoreCoeff,
            double springCoeff) :
            this(triangleIndex, shapePoint, softConstraints, mass, decompositionParam, restoreCoeff, springCoeff, 0.0, 0.0)
        { }

        public SoftShape(
            TriangleMesh[] triangleIndex,
            Vector3[] shapePoint,
            Vector3 startPosition,
            double mass,
            double decompositionParam,
            double errorReductionParam,
            double springCoeff,
            double angularErrorReductionParam,
            double angularSpringCoeff)
        {
            Triangle = triangleIndex;

            DecompositionParameter = decompositionParam;
            Mass = mass;
            InertiaTensor = Matrix3x3.IdentityMatrix();

            Position = startPosition;

            AddSoftShapePoint(shapePoint);
            BuildSoftConstraints(errorReductionParam, springCoeff, angularErrorReductionParam, angularSpringCoeff);
                        
            SleepingFrameCount = 0;

            SetAABB();
        }

        public SoftShape(
            TriangleMesh[] triangleIndex,
            Vector3[] shapePoint,
            Vector3 startPosition,
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
            Mass = mass;
        }
               
        public override void SetAABB()
        {
            AABBox = AABB.GetShapePointAABB(ShapePoints, null);
        }

        public void SetPointsMass(double mass)
        {
            Mass = mass;
            
            if (IsStatic)
            {
                Mass = 0.0;
                InverseMass = 0.0;
            }
            else if (Mass > 0.0)
                InverseMass = 1.0 / Mass;
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

        public override void Rotate(Vector3 versor, double angle)
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


        #endregion

        #region Private Methods

        private void AddSoftShapePoint(
            Vector3[] points)
        {
            ShapePoints = new SoftShapePoint[points.Length];

            double mass = Mass * (1.0 / points.Length);
            double inverseMass = 1.0 / mass;

            Matrix3x3 inertiaTensor = Matrix3x3.IdentityMatrix() *
                                      (diameter * diameter * 0.4);

            for (int i = 0; i < points.Length; i++)
            {
                ShapePoints[i] = new SoftShapePoint(diameter);
                ShapePoints[i].SetStartPosition(points[i]);
                ShapePoints[i].SetPosition(points[i] + Position);

                Vector3 r = -1.0 * ShapePoints[i].StartPosition;
                Matrix3x3 baseTensors = Matrix3x3.Invert(
                                inertiaTensor +
                                (Matrix3x3.IdentityMatrix() * r.Dot(r) - Matrix3x3.OuterProduct(r, r)) *
                                mass);

                ShapePoints[i].SetLinearVelocity(LinearVelocity);
                ShapePoints[i].SetAngularVelocity(AngularVelocity);
                ShapePoints[i].SetMass(mass);
                ShapePoints[i].SetInverseMass(inverseMass);
                ShapePoints[i].SetBaseInertiaTensor(baseTensors);
                ShapePoints[i].SetInertiaTensor(baseTensors);
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
