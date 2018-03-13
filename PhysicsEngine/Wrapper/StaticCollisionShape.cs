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

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class StaticCollisionShape : ICollisionShape, IMapper
    {
        #region Fileds

        ConvexShape convexShape;

        #endregion

        #region Constructor

        public StaticCollisionShape()
        {
            convexShape = new ConvexShape(ObjectType.StaticBody);
        }

        #endregion

        #region Public Methods

        IShape IMapper.GetShape()
        {
            return convexShape;
        }

        public void SetGeometry(
            Vector3[] inputVertexPosition,
            TriangleIndexes[] inputTriangle)
        {
            convexShape.SetGeometry(new Geometry(convexShape, inputVertexPosition, inputTriangle, ObjectGeometryType.ConvexBody, true));
        }

        public Vector3 AngularVelocity
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
                return convexShape.BaseInertiaTensor;
            }
        }

        public Vector3 ForceValue
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
                return convexShape.InertiaTensor;
            }
        }

        public double InverseMass
        {
            get
            {
                return convexShape.InverseMass;
            }
        }

        public Vector3 LinearVelocity
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
                return convexShape.Mass;
            }
        }

        public ObjectType ObjectType
        {
            get
            {
                return convexShape.ObjectType;
            }
        }

        public Vector3 Position
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

        public Vector3 StartPosition
        {
            get
            {
                return convexShape.StartPosition;
            }
        }

        public int GetID()
        {
            return convexShape.ID;
        }

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            convexShape.SetAngularVelocity(inputAngularVelocity);
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            convexShape.SetBaseInertiaTensor(inputIntertiaTensor);
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            convexShape.SetDynamicFrictionCoeff(dynamicFrictionCoeff);
        }

        public void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            convexShape.SetExcludeFromCollisionDetection(excludeFromCollisionDetection);
        }

        public void SetForce(Vector3 force)
        {
            convexShape.SetForce(force);
        }

        public void SetInertiaTensor(Matrix3x3 inertiaTensor)
        {
            convexShape.SetInertiaTensor(inertiaTensor);
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            convexShape.SetLinearVelocity(inputLinearVelocity);
        }

        public void SetMass(double mass)
        {
            convexShape.SetMass(mass);
        }

        public void SetPosition(Vector3 inputPosition)
        {
            convexShape.SetPosition(inputPosition);
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            convexShape.SetRestitutionCoeff(restitutionCoeff);
        }

        public void SetRestoreCoeff(double value)
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

        public void SetTorque(Vector3 torque)
        {
            convexShape.SetTorque(torque);
        }

        public int GetGeometryCount()
        {
            return 1;
        }

        public Vector3 GetMinAABB()
        {
            return convexShape.ObjectGeometry.AABBox.Min;
        }

        public Vector3 GetMaxAABB()
        {
            return convexShape.ObjectGeometry.AABBox.Max;
        }

        #endregion
    }
}
