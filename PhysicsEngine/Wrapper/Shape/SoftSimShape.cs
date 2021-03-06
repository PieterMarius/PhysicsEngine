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
using SharpEngineMathUtility;
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class SoftShape : ICollisionShape, IMapper
    {
        #region Fields

        SimSoftShape softShape;

        #endregion

        #region Constructor

        public SoftShape(
            int[][] triangleIndex,
            Vector3d[] shapePoint,
            Vector3d startPosition,
            double mass,
            double decompositionParam,
            double dampingCoefficient,
            double springCoeff,
            double angularErrorReductionParam,
            double angularSpringCoeff)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(triangleIndex);

            softShape = new SimSoftShape(
                triangleMeshes,
                shapePoint,
                startPosition,
                mass,
                decompositionParam,
                dampingCoefficient,
                springCoeff,
                angularErrorReductionParam,
                angularSpringCoeff);
        }

        public SoftShape(
            int[][] triangleIndex,
            Vector3d[] shapePoint,
            Vector3d startPosition,
            double mass,
            double decompositionParam,
            double dampingCoefficient,
            double springCoefficient)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(triangleIndex);

            softShape = new SimSoftShape(
                triangleMeshes, 
                shapePoint, 
                startPosition, 
                mass, 
                decompositionParam, 
                dampingCoefficient, 
                springCoefficient);
        }

        public SoftShape(
            int[][] triangleIndex,
            Vector3d[] shapePoint,
            ConstraintIndex[] softJoint,
            double mass,
            double decompositionParam,
            double restoreCoefficient,
            double springCoefficient)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(triangleIndex);

            softShape = new SimSoftShape(triangleMeshes, shapePoint, softJoint, mass, decompositionParam, restoreCoefficient, springCoefficient);
        }

        #endregion

        #region Public Methods

        public void SetGeometry(
            Vector3d[] inputVertexPosition,
            int[][] inputTriangle)
        {
            throw new NotImplementedException();
        }

        public Vector3d AngularVelocity
        {
            get
            {
                return softShape.AngularVelocity;
            }
        }

        public Matrix3x3 BaseInertiaTensor
        {
            get
            {
                return softShape.MassInfo.InverseBaseInertiaTensor;
            }
        }

        public Vector3d ForceValue
        {
            get
            {
                return softShape.ForceValue;
            }
        }

        public Matrix3x3 InertiaTensor
        {
            get
            {
                return softShape.MassInfo.InverseInertiaTensor;
            }
        }

        public double InverseMass
        {
            get
            {
                return softShape.MassInfo.InverseMass;
            }
        }

        public Vector3d LinearVelocity
        {
            get
            {
                return softShape.LinearVelocity;
            }
        }

        public double Mass
        {
            get
            {
                return softShape.MassInfo.Mass;
            }
        }

        public ObjectType ObjectType
        {
            get
            {
                return softShape.ObjectType;
            }
        }

        public Vector3d Position
        {
            get
            {
                return softShape.Position;
            }
        }

        public Matrix3x3 RotationMatrix
        {
            get
            {
                return softShape.RotationMatrix;
            }
        }

        public Vector3d InitCenterOfMass
        {
            get
            {
                return softShape.InitCenterOfMass;
            }
        }

        public bool IsStatic
        {
            get
            {
                return softShape.IsStatic;
            }
        }

        public void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            softShape.SetExcludeFromCollisionDetection(excludeFromCollisionDetection);
        }

        public int GetID()
        {
            return softShape.ID;
        }

        public void SetAngularVelocity(Vector3d inputAngularVelocity)
        {
            throw new NotSupportedException();
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            throw new NotImplementedException();
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            softShape.SetDynamicFrictionCoeff(dynamicFrictionCoeff);
        }

        public void SetForce(Vector3d force)
        {
            throw new NotImplementedException();
        }

        public void SetInverseInertiaTensor(Matrix3x3 inertiaTensor)
        {
            throw new NotImplementedException();
        }

        public void SetLinearVelocity(Vector3d inputLinearVelocity)
        {
            throw new NotImplementedException();
        }

        public void SetMass(double mass)
        {
            softShape.SetMass(mass);
        }

        public void SetPosition(Vector3d inputPosition)
        {
            
            throw new NotSupportedException();
        }

        public void SetIsStatic(bool isStatic)
        {

            throw new NotSupportedException();
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            softShape.SetRestitutionCoeff(restitutionCoeff);
        }

        public void SetErrorReductionParam(double value)
        {
            softShape.SetRestoreCoeff(value);
        }

        public void SetConstraintsRestoreCoeff(double value)
        {
            softShape.SetConstraintsErrorReductionParam(value);
        }

        public void SetConstraintsSpringCoeff(double value)
        {
            softShape.SetConstraintsSpringCoefficient(value);
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            throw new NotImplementedException();
        }

        public void SetRotationStatus(Quaternion inputRotationStatus)
        {
            throw new NotImplementedException();
        }

        public void SetSleepingFrameCount(int frameCount)
        {
            throw new NotImplementedException();
        }

        public void SetStaticFrictionCoeff(double staticFrictionCoeff)
        {
            softShape.SetStaticFrictionCoeff(staticFrictionCoeff);
        }

        public void SetTorque(Vector3d torque)
        {
            throw new NotImplementedException();
        }

        IShape IMapper.GetShape()
        {
            return softShape;
        }

        public int GetGeometryCount()
        {
            return 0;
        }

        public Vector3d GetMinAABB()
        {
            return softShape.AABBox.Min;
        }

        public Vector3d GetMaxAABB()
        {
            return softShape.AABBox.Max;
        }
    
        public int GetShapePointsCount()
        {
            return softShape.ShapePoints.Length;
        }

        public Vector3d[] GetVertices()
        {
            return Array.ConvertAll(softShape.ShapePoints, x => x.Position);
        }

        public int GetShapeConstraintsCount()
        {
            return softShape.SoftConstraint.Count;
        }

        public Vector3d[] GetShapeConstraintsPosition()
        {
            return softShape.SoftConstraint.Select(x => x.GetAnchorPosition()).ToArray();
        }

        public double[] GetShapeErrorReductionParams()
        {
            return softShape.SoftConstraint.Select(x => x.GetErrorReductionParam()).ToArray();
        }

        public void AddToConstraintsRestoreCoefficient(double value)
        {
            softShape.AddToConstraintsErrorReductionParam(value);
        }

        public void AddToConstraintsSpringCoefficient(double value)
        {
            softShape.AddToConstraintsSpringCoefficient(value);
        }

        public double GetDecompositionParam()
        {
            return softShape.DecompositionParameter;
        }

        public Vector3d GetCenterOfMassShiftValue(int index = 0)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
