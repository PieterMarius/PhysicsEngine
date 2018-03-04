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

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal abstract class Shape : IShape, IDentity
    {
        #region Fields

        public double RestoreCoeff { get; protected set; }

        public double DynamicFrictionCoeff { get; protected set; }

        public bool ExcludeFromCollisionDetection { get; protected set; }

        public double RestitutionCoeff { get; protected set; }

        public Quaternion RotationStatus { get; protected set; }

        public int SleepingFrameCount { get; protected set; }

        public Vector3 StartPosition { get; protected set; }

        public double StaticFrictionCoeff { get; protected set; }

        public Vector3 TorqueValue { get; protected set; }

        public int ID { get; protected set; }

        public ObjectType ObjectType { get; protected set; }

        public Vector3 Position { get; protected set; }

        public Vector3 LinearVelocity { get; protected set; }

        public Vector3 AngularVelocity { get; protected set; }

        public Matrix3x3 InertiaTensor { get; protected set; }

        public Matrix3x3 BaseInertiaTensor { get; protected set; }

        public Matrix3x3 RotationMatrix { get; protected set; }

        public double Mass { get; protected set; }

        public double InverseMass { get; protected set; }

        public Vector3 TempAngularVelocity { get; protected set; }

        public Vector3 TempLinearVelocity { get; protected set; }

        public Vector3 ForceValue { get; protected set; }

        #endregion

        #region Public Methods

        public abstract void Rotate(Vector3 versor, double angle);

        public abstract void SetAABB();

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            AngularVelocity = inputAngularVelocity;
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            BaseInertiaTensor = Matrix3x3.Invert(inputIntertiaTensor);
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            DynamicFrictionCoeff = dynamicFrictionCoeff;
        }

        public void SetExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            ExcludeFromCollisionDetection = excludeFromCollisionDetection;
        }

        public void SetForce(Vector3 force)
        {
            ForceValue = force;
        }

        public void SetID(int id)
        {
            ID = id;
        }

        public void SetInertiaTensor(Matrix3x3 inertiaTensor)
        {
            InertiaTensor = inertiaTensor;
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            LinearVelocity = inputLinearVelocity;
        }

        public abstract void SetMass(double mass);

        public void SetPosition(Vector3 inputPosition)
        {
            Position = inputPosition;
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            RestitutionCoeff = restitutionCoeff;
        }

        public void SetRestoreCoeff(double value)
        {
            RestoreCoeff = value;
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            RotationMatrix = inputRotationMatrix;
        }

        public void SetRotationStatus(Quaternion inputRotationStatus)
        {
            RotationStatus = inputRotationStatus;
        }

        public void SetSleepingFrameCount(int frameCount)
        {
            SleepingFrameCount = frameCount;
        }

        public void SetStaticFrictionCoeff(double staticFrictionCoeff)
        {
            StaticFrictionCoeff = staticFrictionCoeff;
        }

        public void SetTempAngularVelocity(Vector3 inputAngularVelocity)
        {
            TempAngularVelocity = inputAngularVelocity;
        }

        public void SetTempLinearVelocity(Vector3 inputLinearVelocity)
        {
            TempLinearVelocity = inputLinearVelocity;
        }

        public void SetTorque(Vector3 torque)
        {
            TorqueValue = torque;
        }

        #endregion
    }
}
