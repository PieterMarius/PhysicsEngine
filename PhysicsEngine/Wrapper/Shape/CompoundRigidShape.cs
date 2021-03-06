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
using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class CompoundRigidShape : ICollisionShape, IMapper
    {
        #region Fields

        CompoundShape compoundShape;

        #endregion

        #region Constructor

        public CompoundRigidShape(
            List<Vector3d[]> inputVertexPosition,
            List<int[][]> inputTriangle,
            Vector3d[] compoundPosition,
            double[] mass)
        {
            compoundShape = new CompoundShape(
                inputVertexPosition, 
                inputTriangle,
                compoundPosition,
                mass);
        }

        #endregion

        #region Public Methods


        public void SetPartialMass(double[] mass)
        {
            compoundShape.SetPartialMass(mass);
        }

        public void SetCompoundPosition(Vector3d[] compoundPosition)
        {
            compoundShape.SetCompoundPosition(compoundPosition);
        }

        public Vector3d[] StartCompoundPositionObjects
        {
            get
            {
                return compoundShape.StartCompoundPositionObjects;
            }
        }


        public Vector3d AngularVelocity
        {
            get
            {
                return compoundShape.AngularVelocity;
            }
        }

        public Matrix3x3 BaseInertiaTensor
        {
            get
            {
                return compoundShape.MassInfo.InverseBaseInertiaTensor;
            }
        }

        public Vector3d ForceValue
        {
            get
            {
                return compoundShape.ForceValue;
            }
        }

        public Matrix3x3 InertiaTensor
        {
            get
            {
                return compoundShape.MassInfo.InverseInertiaTensor;
            }
        }

        public double InverseMass
        {
            get
            {
                return compoundShape.MassInfo.InverseMass;
            }
        }

        public Vector3d LinearVelocity
        {
            get
            {
                return compoundShape.LinearVelocity;
            }
        }

        public double Mass
        {
            get
            {
                return compoundShape.MassInfo.Mass;
            }
        }

        public ObjectType ObjectType
        {
            get
            {
                return compoundShape.ObjectType;
            }
        }

        public Vector3d Position
        {
            get
            {
                return compoundShape.Position;
            }
        }

        public Matrix3x3 RotationMatrix
        {
            get
            {
                return compoundShape.RotationMatrix;
            }
        }

        public Vector3d InitCenterOfMass
        {
            get
            {
                return compoundShape.InitCenterOfMass;
            }
        }

        public Vector3d[] GetVertices()
        {
            List<Vector3d> vertices = new List<Vector3d>();
            for (int i = 0; i < compoundShape.CompoundingConvexObjCount; i++)
            {
                for (int j = 0; j < compoundShape.ShapesGeometry[i].BaseGeometry.VerticesIdx.Length; j++)
                    vertices.Add(CommonUtilities.GetVertexPosition(
                        compoundShape.ShapesGeometry[i], 
                        compoundShape.ShapesGeometry[i].BaseGeometry.VerticesIdx[j].ID).Vertex);
            }

            return vertices.ToArray();
        }

        public bool IsStatic
        {
            get
            {
                return compoundShape.IsStatic;
            }
        }

        public void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            compoundShape.SetExcludeFromCollisionDetection(excludeFromCollisionDetection);
        }

        public int GetID()
        {
            return compoundShape.ID;
        }

        IShape IMapper.GetShape()
        {
            return compoundShape;
        }

        public void SetAngularVelocity(Vector3d inputAngularVelocity)
        {
            compoundShape.SetAngularVelocity(inputAngularVelocity);
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            compoundShape.SetInverseBaseInertiaTensor(inputIntertiaTensor);
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            compoundShape.SetDynamicFrictionCoeff(dynamicFrictionCoeff);
        }

        public void SetForce(Vector3d force)
        {
            compoundShape.SetForce(force);
        }

        public void SetInverseInertiaTensor(Matrix3x3 inertiaTensor)
        {
            compoundShape.SetInverseInertiaTensor(inertiaTensor);
        }

        public void SetLinearVelocity(Vector3d inputLinearVelocity)
        {
            compoundShape.SetLinearVelocity(inputLinearVelocity);
        }

        public void SetMass(double mass)
        {
            compoundShape.SetMass(mass);
        }

        public void SetPosition(Vector3d inputPosition)
        {
            compoundShape.SetPosition(inputPosition);
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            compoundShape.SetRestitutionCoeff(restitutionCoeff);
        }

        public void SetErrorReductionParam(double value)
        {
            compoundShape.SetRestoreCoeff(value);
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            compoundShape.SetRotationMatrix(inputRotationMatrix);
        }

        public void SetRotationStatus(Quaternion inputRotationStatus)
        {
            compoundShape.SetRotationStatus(inputRotationStatus);
        }

        public void SetSleepingFrameCount(int frameCount)
        {
            compoundShape.SetSleepingFrameCount(frameCount);
        }

        public void SetStaticFrictionCoeff(double staticFrictionCoeff)
        {
            compoundShape.SetStaticFrictionCoeff(staticFrictionCoeff);
        }

        public void SetTorque(Vector3d torque)
        {
            compoundShape.SetTorque(torque);
        }

        public void SetIsStatic(bool isStatic)
        {
            compoundShape.SetIsStatic(isStatic);
        }
        
        public int GetGeometryCount()
        {
            return compoundShape.ShapesGeometry.Length;
        }

        public Vector3d GetMinAABB()
        {
            return CommonUtilities.GetAABBMinValue(compoundShape.ShapesGeometry);
        }

        public Vector3d GetMaxAABB()
        {
            return CommonUtilities.GetAABBMaxValue(compoundShape.ShapesGeometry);
        }

        public void AddToRestoreCoeff(double value)
        {
            throw new NotImplementedException();
        }

        public Vector3d GetCenterOfMassShiftValue(int index = 0)
        {
            return compoundShape.StartCompoundPositionObjects[index] - compoundShape.InitCenterOfMass;
        }

        #endregion
    }
}
