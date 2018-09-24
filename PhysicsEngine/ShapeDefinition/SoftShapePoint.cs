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

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal class SoftShapePoint: IShapeCommon
    {
        #region Fields

        public int ID { get; private set; }
        public Vector3d Position { get; private set; }
        public Vector3d StartPosition { get; private set; }
        public Vector3d AngularVelocity { get; private set; }
        public Vector3d LinearVelocity { get; private set; }
        public Matrix3x3 RotationMatrix { get; private set; }
        public Quaternion RotationStatus { get; private set; }
        public double Diameter { get; private set; }
        public MassData MassInfo { get; private set; }
        public ObjectType ObjectType { get; private set; }
        public Vector3d TempAngularVelocity { get; private set; }
        public Vector3d TempLinearVelocity { get; private set; }
        public Vector3d ForceValue { get; private set; }
        public HashSet<int> TriangleIndex { get; private set; }
        public bool IsStatic { get; private set; }

        #endregion

        #region Constructor

        public SoftShapePoint(double diameter)
        {
            Diameter = diameter;
            RotationMatrix = Matrix3x3.IdentityMatrix();
            TriangleIndex = new HashSet<int>();
            ObjectType = ObjectType.SoftPoint;
        }

        #endregion

        #region Public Methods

        public void SetID(int id)
        {
            ID = id;
        }

        public void SetMass(double mass)
        {
            MassInfo.Mass = mass;
        }

        public void SetPosition(Vector3d position)
        {
            Position = position;
        }

        public void SetAngularVelocity(Vector3d angularVelocity)
        {
            AngularVelocity = angularVelocity;
        }

        public void SetLinearVelocity(Vector3d linearVelocity)
        {
            LinearVelocity = linearVelocity;
        }

        public void SetTempAngularVelocity(Vector3d angularVelocity)
        {
            TempAngularVelocity = angularVelocity;
        }

        public void SetTempLinearVelocity(Vector3d linearVelocity)
        {
            TempLinearVelocity = linearVelocity;
        }

        public void SetInverseInertiaTensor(Matrix3x3 inertiaTensor)
        {
            //TODO verificare inverse
            MassInfo.InertiaTensor = inertiaTensor;
        }

        public void SetInverseMass(double inverseMass)
        {
            MassInfo.InverseMass = inverseMass;
        }

        public void SetInverseBaseInertiaTensor(Matrix3x3 inertiaTensor)
        {
            MassInfo.InverseBaseInertiaTensor = inertiaTensor;
        }

        public void SetForce(Vector3d force)
        {
            ForceValue = force;
        }

        public void AddTrianglesIndex(int triangleIndexes)
        {
            TriangleIndex.Add(triangleIndexes);
        }

        public void SetRotationStatus(Quaternion rotationStatus)
        {
            RotationStatus = rotationStatus;
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            RotationMatrix = inputRotationMatrix;
        }

        public void SetStartPosition(Vector3d startPosition)
        {
            StartPosition = startPosition;
        }

        public void SetIsStatic(bool isStatic)
        {
            IsStatic = isStatic;
        }

        #endregion
    }
}
