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
using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class CompoundRigidCollisionShape : ICollisionShape, IMapper
    {
        #region Fields

        CompoundShape compoundShape;

        #endregion

        #region Constructor

        public CompoundRigidCollisionShape()
        {
            compoundShape = new CompoundShape();
        }

        #endregion

        #region Public Methods


        public void SetPartialMass(double[] mass)
        {
            compoundShape.SetPartialMass(mass);
        }

        public void SetCompoundPosition(Vector3[] compoundPosition)
        {
            compoundShape.SetCompoundPosition(compoundPosition);
        }

        public void SetGeometry(
            List<Vector3[]> inputVertexPosition,
            List<int[][]> inputTriangle)
        {
            IGeometry[] geometry = new IGeometry[inputVertexPosition.Count];

            for (int i = 0; i < inputVertexPosition.Count; i++)
            {
                TriangleMesh[] triangleMeshes = WrapperUtilities.GetTriangleMeshes(inputTriangle[i]);

                geometry[i] = new Geometry(compoundShape, inputVertexPosition[i], triangleMeshes, ObjectGeometryType.ConvexBody, true);
            }

            compoundShape.SetObjectGeometry(geometry);
        }

        public Vector3[] StartCompoundPositionObjects
        {
            get
            {
                return compoundShape.StartCompoundPositionObjects;
            }
        }


        public Vector3 AngularVelocity
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
                return compoundShape.BaseInertiaTensor;
            }
        }

        public Vector3 ForceValue
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
                return compoundShape.InertiaTensor;
            }
        }

        public double InverseMass
        {
            get
            {
                return compoundShape.InverseMass;
            }
        }

        public Vector3 LinearVelocity
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
                return compoundShape.Mass;
            }
        }

        public ObjectType ObjectType
        {
            get
            {
                return compoundShape.ObjectType;
            }
        }

        public Vector3 Position
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

        public Vector3 InitCenterOfMass
        {
            get
            {
                return compoundShape.InitCenterOfMass;
            }
        }

        //public Vector3[] GetVertices()
        //{
        //    List<Vector3> vertices = new List<Vector3>();
        //    for (int i = 0; i < compoundShape.CompoundingConvexObjectCount; i++)
        //    {
        //        for (int j = 0; j < compoundShape.ge; j++)
        //        {

        //        }
        //    }
        //}

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

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            compoundShape.SetAngularVelocity(inputAngularVelocity);
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            compoundShape.SetBaseInertiaTensor(inputIntertiaTensor);
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            compoundShape.SetDynamicFrictionCoeff(dynamicFrictionCoeff);
        }

        public void SetForce(Vector3 force)
        {
            compoundShape.SetForce(force);
        }

        public void SetInertiaTensor(Matrix3x3 inertiaTensor)
        {
            compoundShape.SetInertiaTensor(inertiaTensor);
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            compoundShape.SetLinearVelocity(inputLinearVelocity);
        }

        public void SetMass(double mass)
        {
            compoundShape.SetMass(mass);
        }

        public void SetPosition(Vector3 inputPosition)
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

        public void SetTorque(Vector3 torque)
        {
            compoundShape.SetTorque(torque);
        }

        public void SetIsStatic(bool isStatic)
        {
            compoundShape.SetIsStatic(isStatic);
        }

        public void SetGeometry(Vector3[] inputVertexPosition, int[][] inputTriangle)
        {
            throw new NotImplementedException();
        }

        public int GetGeometryCount()
        {
            return compoundShape.ObjectGeometry.Length;
        }

        public Vector3 GetMinAABB()
        {
            double xMin = double.MaxValue;
            double yMin = double.MaxValue;
            double zMin = double.MaxValue;
            
            foreach (var item in compoundShape.ObjectGeometry)
            {
                if (item.AABBox.Min.x < xMin)
                    xMin = item.AABBox.Min.x;
                if (item.AABBox.Min.y < yMin)
                    yMin = item.AABBox.Min.y;
                if (item.AABBox.Min.z < zMin)
                    zMin = item.AABBox.Min.z;
            }

            return new Vector3(xMin, yMin, zMin);
        }

        public Vector3 GetMaxAABB()
        {
            double xMax = double.MinValue;
            double yMax = double.MinValue;
            double zMax = double.MinValue;
            
            foreach (var item in compoundShape.ObjectGeometry)
            {
                if (item.AABBox.Max.x > xMax)
                    xMax = item.AABBox.Max.x;
                if (item.AABBox.Max.y < yMax)
                    yMax = item.AABBox.Max.y;
                if (item.AABBox.Max.z < zMax)
                    zMax = item.AABBox.Max.z;
            }

            return new Vector3(xMax, yMax, zMax);
        }

        public void AddToRestoreCoeff(double value)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
