﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject
{
    public sealed class CompoundRigidCollisionShape : ICollisionShape, IMapper
    {
        #region Fields

        CompoundShape compoundShape;

        #endregion

        #region Constructor

        public CompoundRigidCollisionShape()
        {
            compoundShape = new CompoundShape(ObjectType.CompoundShape);
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
            List<TriangleIndexes[]> inputTriangle)
        {
            IGeometry[] geometry = new IGeometry[inputVertexPosition.Count];

            for (int i = 0; i < inputVertexPosition.Count; i++)
            {
                geometry[i] = new Geometry(compoundShape, inputVertexPosition[i], inputTriangle[i], ObjectGeometryType.ConvexBody, true);
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

        public Vector3 StartPosition
        {
            get
            {
                return compoundShape.StartPosition;
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

        public void SetRestoreCoeff(double value)
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

        public void SetGeometry(Vector3[] inputVertexPosition, TriangleIndexes[] inputTriangle)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
