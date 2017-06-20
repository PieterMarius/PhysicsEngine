﻿using System;
using SharpEngineMathUtility;
using System.Collections.Generic;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public class SoftShapePoint: IShapeCommon, Identity
    {
        #region Fields

        int Identity.ID { get; set; }
        public double Mass { get; private set; }
        public Vector3 Position { get; private set; }
        public Vector3 StartPosition { get; private set; }
        public Vector3 AngularVelocity { get; private set; }
        public Vector3 LinearVelocity { get; private set; }
        public Matrix3x3 RotationMatrix { get; private set; }
        public Quaternion RotationStatus { get; private set; }
        public double Diameter { get; private set; }
        public Matrix3x3 InertiaTensor { get; private set; }
        public double InverseMass { get; private set; }
        public ObjectType ObjectType { get; private set; }
        public Vector3 TempAngularVelocity { get; private set; }
        public Vector3 TempLinearVelocity { get; private set; }
        public Matrix3x3 BaseInertiaTensor { get; private set; }
        public Vector3 ForceValue { get; private set; }
        public int[] TriangleIndex { get; private set; }

        #endregion

        #region Constructor

        public SoftShapePoint(double diameter)
        {
            Diameter = diameter;
            RotationMatrix = Matrix3x3.IdentityMatrix();
        }

        #endregion

        #region Public Methods

        public int GetID()
        {
            return ((Identity)this).ID;
        }

        public void SetMass(double mass)
        {
            Mass = mass;
        }

        public void SetPosition(Vector3 position)
        {
            Position = position;
        }

        public void SetAngularVelocity(Vector3 angularVelocity)
        {
            AngularVelocity = angularVelocity;
        }

        public void SetLinearVelocity(Vector3 linearVelocity)
        {
            LinearVelocity = linearVelocity;
        }

        public void SetTempAngularVelocity(Vector3 angularVelocity)
        {
            TempAngularVelocity = angularVelocity;
        }

        public void SetTempLinearVelocity(Vector3 linearVelocity)
        {
            TempLinearVelocity = linearVelocity;
        }

        public void SetInertiaTensor(Matrix3x3 inertiaTensor)
        {
            InertiaTensor = inertiaTensor;
        }

        public void SetInverseMass(double inverseMass)
        {
            InverseMass = inverseMass;
        }

        public void SetBaseInertiaTensor(Matrix3x3 inertiaTensor)
        {
            BaseInertiaTensor = inertiaTensor;
        }

        public void SetForce(Vector3 force)
        {
            ForceValue = force;
        }

        public void SetTrianglesIndex(int[] triangleIndexes)
        {
            TriangleIndex = triangleIndexes;
        }

        #endregion
    }
}
