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
    public interface ICollisionShape
    {
        ObjectType ObjectType { get; }
        Vector3d Position { get; }
        Vector3d InitCenterOfMass { get; }
        Vector3d LinearVelocity { get; }
        Vector3d AngularVelocity { get; }
        Matrix3x3 InertiaTensor { get; }
        Matrix3x3 BaseInertiaTensor { get; }
        Matrix3x3 RotationMatrix { get; }
        double Mass { get; }
        double InverseMass { get; }
        Vector3d ForceValue { get; }
        bool IsStatic { get; }

        int GetID();
        int GetGeometryCount();
        Vector3d GetMinAABB();
        Vector3d GetMaxAABB();
        Vector3d[] GetVertices();
        Vector3d GetCenterOfMassShiftValue(int index = 0);
        void SetMass(double mass);
        void SetPosition(Vector3d inputPosition);
        void SetLinearVelocity(Vector3d inputLinearVelocity);
        void SetAngularVelocity(Vector3d inputAngularVelocity);
        void SetInverseInertiaTensor(Matrix3x3 inertiaTensor);
        void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor);
        void SetForce(Vector3d force);
        void SetRotationMatrix(Matrix3x3 inputRotationMatrix);
        void SetErrorReductionParam(double value);
        void SetDynamicFrictionCoeff(double dynamicFrictionCoeff);
        void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection);
        void SetRestitutionCoeff(double restitutionCoeff);
        void SetRotationStatus(Quaternion inputRotationStatus);
        void SetSleepingFrameCount(int frameCount);
        void SetStaticFrictionCoeff(double staticFrictionCoeff);
        void SetTorque(Vector3d torque);
        void SetIsStatic(bool isStatic);
    }
}
