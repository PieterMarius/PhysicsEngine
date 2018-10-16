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

using SharpPhysicsEngine.ShapeDefinition;
using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ConvexHullWrapper;
using SharpPhysicsEngine.Helper;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class ConvexShape: ICollisionShape, IMapper
    {
        #region Fields

        ShapeDefinition.ConvexShape convexShape;

        #endregion

        #region Constructor

        public ConvexShape(
            Vector3d[] inputVertexPosition,
            int[][] inputTriangle,
            Vector3d position,
            double mass,
            bool isStatic)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(inputTriangle);
            ShapeGeometry baseGeometry = new ShapeGeometry(inputVertexPosition, inputTriangle);

            convexShape = new ShapeDefinition.ConvexShape(
                baseGeometry.GetGeometry(),
                position,
                mass, 
                isStatic);
        }
        public ConvexShape(
            Vector3d[] inputVertexPosition,
            int[][] inputTriangle,
            Vector3d position,
            double mass) :
            this(inputVertexPosition, inputTriangle, position, mass, false)
        { }

        public ConvexShape(
            Vector3d[] inputVertexPosition,
            Vector3d position,
            double mass,
            bool isStatic)
        {
            ShapeGeometry baseGeometry = new ShapeGeometry(inputVertexPosition);

            convexShape = new ShapeDefinition.ConvexShape(
                baseGeometry.GetGeometry(),
                position,
                mass,
                isStatic);
        }

        public ConvexShape(
            Vector3d[] inputVertexPosition,
            Vector3d position,
            double mass) :
            this(inputVertexPosition, position, mass, false)
        { }

        public ConvexShape(
            ShapeGeometry baseGeometry, 
            Vector3d position,
            double mass,
            bool isStatic)
        {
            convexShape = new ShapeDefinition.ConvexShape(
               baseGeometry.GetGeometry(),
               position,
               mass,
               isStatic);
        }

        #endregion

        #region Public Methods

        IShape IMapper.GetShape()
        {
            return convexShape;
        }
                
        public Vector3d AngularVelocity
        {
            get
            {
                return convexShape.AngularVelocity;
            }
        }

        public Matrix3x3 BaseInertiaTensor
        {
            get
            {
                return convexShape.MassInfo.InverseBaseInertiaTensor;
            }
        }

        public Vector3d ForceValue
        {
            get
            {
                return convexShape.ForceValue;
            }
        }

        public Matrix3x3 InertiaTensor
        {
            get
            {
                return convexShape.MassInfo.InverseInertiaTensor;
            }
        }

        public double InverseMass
        {
            get
            {
                return convexShape.MassInfo.InverseMass;
            }
        }

        public Vector3d LinearVelocity
        {
            get
            {
                return convexShape.LinearVelocity;
            }
        }

        public double Mass
        {
            get
            {
                return convexShape.MassInfo.Mass;
            }
        }

        public ObjectType ObjectType
        {
            get
            {
                return convexShape.ObjectType;
            }
        }

        public Vector3d Position
        {
            get
            {
                return convexShape.Position;
            }
        }

        public Matrix3x3 RotationMatrix
        {
            get
            {
                return convexShape.RotationMatrix;
            }
        }

        public Vector3d InitCenterOfMass
        {
            get
            {
                return convexShape.InitCenterOfMass;
            }
        }

        public bool IsStatic
        {
            get
            {
                return convexShape.IsStatic;
            }
        }

        public int GetID()
        {
            return convexShape.ID;
        }

        public void SetAngularVelocity(Vector3d inputAngularVelocity)
        {
            convexShape.SetAngularVelocity(inputAngularVelocity);
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            convexShape.SetInverseBaseInertiaTensor(inputIntertiaTensor);
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            convexShape.SetDynamicFrictionCoeff(dynamicFrictionCoeff);
        }

        public void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            convexShape.SetExcludeFromCollisionDetection(excludeFromCollisionDetection);
        }

        public void SetForce(Vector3d force)
        {
            convexShape.SetForce(force);
        }

        public void SetInverseInertiaTensor(Matrix3x3 inertiaTensor)
        {
            convexShape.SetInverseInertiaTensor(inertiaTensor);
        }

        public void SetLinearVelocity(Vector3d inputLinearVelocity)
        {
            convexShape.SetLinearVelocity(inputLinearVelocity);
        }

        public void SetMass(double mass)
        {
            convexShape.SetMass(mass);
        }

        public void SetPosition(Vector3d inputPosition)
        {
            convexShape.SetPosition(inputPosition);
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            convexShape.SetRestitutionCoeff(restitutionCoeff);
        }

        public void SetErrorReductionParam(double value)
        {
            convexShape.SetRestoreCoeff(value);
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            convexShape.SetRotationMatrix(inputRotationMatrix);
        }

        public void SetRotationStatus(Quaternion inputRotationStatus)
        {
            convexShape.SetRotationStatus(inputRotationStatus);
        }

        public void SetSleepingFrameCount(int frameCount)
        {
            convexShape.SetSleepingFrameCount(frameCount);
        }

        public void SetStaticFrictionCoeff(double staticFrictionCoeff)
        {
            convexShape.SetStaticFrictionCoeff(staticFrictionCoeff);
        }

        public void SetTorque(Vector3d torque)
        {
            convexShape.SetTorque(torque);
        }

        public int GetGeometryCount()
        {
            return 1;
        }

        public Vector3d GetMinAABB()
        {
            return convexShape.ObjectGeometry.AABBox.Min;
        }

        public Vector3d GetMaxAABB()
        {
            return convexShape.ObjectGeometry.AABBox.Max;
        }

        public Vector3d[] GetVertices()
        {
            Vector3d[] vertices = new Vector3d[convexShape.ObjectGeometry.BaseGeometry.VerticesIdx.Length];
            for (int i = 0; i < convexShape.ObjectGeometry.BaseGeometry.VerticesIdx.Length; i++)
                vertices[i] = CommonUtilities.GetVertexPosition(convexShape.ObjectGeometry, i).Vertex;

            return vertices;
        }

        public void SetIsStatic(bool isStatic)
        {
            convexShape.SetIsStatic(isStatic);
        }

        public void AddToRestoreCoeff(double value)
        {
            throw new NotImplementedException();
        }

        public Vector3d GetCenterOfMassShiftValue(int index = 0)
        {
            return -1.0 * convexShape.InitCenterOfMass;
        }

        #endregion
    }
}
