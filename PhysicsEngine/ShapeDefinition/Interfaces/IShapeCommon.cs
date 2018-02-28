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
    internal interface IShapeCommon
    {
        int ID { get; }
        ObjectType ObjectType { get; }
        Vector3 Position { get; }
        Vector3 LinearVelocity { get; }
        Vector3 AngularVelocity { get; }
        Matrix3x3 InertiaTensor { get; }
        Matrix3x3 BaseInertiaTensor { get; }
        Matrix3x3 RotationMatrix { get; }
        double Mass { get; }
        double InverseMass { get; }
        Vector3 TempAngularVelocity { get; }
        Vector3 TempLinearVelocity { get; }
        Vector3 ForceValue { get; }

        void SetMass(double mass);
        void SetPosition(Vector3 inputPosition);
        void SetLinearVelocity(Vector3 inputLinearVelocity);
        void SetAngularVelocity(Vector3 inputAngularVelocity);
        void SetInertiaTensor(Matrix3x3 inertiaTensor);
        void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor);
        void SetForce(Vector3 force);
        void SetTempAngularVelocity(Vector3 inputAngularVelocity);
        void SetTempLinearVelocity(Vector3 inputLinearVelocity);
        void SetRotationMatrix(Matrix3x3 inputRotationMatrix);
    }
}
