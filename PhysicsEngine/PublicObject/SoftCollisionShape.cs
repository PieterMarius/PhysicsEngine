using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject
{
    public sealed class SoftCollisionShape : ICollisionShape, IMapper
    {
        #region Fields

        SoftShape softShape;

        #endregion


        #region Constructor

        public SoftCollisionShape()
        {
            //softShape = new SoftShape();
        }

        #endregion

        #region Public Methods

        public void SetGeometry(
            Vector3[] inputVertexPosition,
            TriangleIndexes[] inputTriangle)
        {
            throw new NotImplementedException();
        }

        public Vector3 AngularVelocity
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Matrix3x3 BaseInertiaTensor
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Vector3 ForceValue
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Matrix3x3 InertiaTensor
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public double InverseMass
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Vector3 LinearVelocity
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public double Mass
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public ObjectType ObjectType
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Vector3 Position
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Matrix3x3 RotationMatrix
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public Vector3 StartPosition
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            throw new NotImplementedException();
        }

        public int GetID()
        {
            throw new NotImplementedException();
        }

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            throw new NotImplementedException();
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            throw new NotImplementedException();
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            throw new NotImplementedException();
        }

        public void SetForce(Vector3 force)
        {
            throw new NotImplementedException();
        }

        public void SetInertiaTensor(Matrix3x3 inertiaTensor)
        {
            throw new NotImplementedException();
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            throw new NotImplementedException();
        }

        public void SetMass(double mass)
        {
            throw new NotImplementedException();
        }

        public void SetPosition(Vector3 inputPosition)
        {
            throw new NotImplementedException();
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            throw new NotImplementedException();
        }

        public void SetRestoreCoeff(double value)
        {
            throw new NotImplementedException();
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            throw new NotImplementedException();
        }

        public void SetRotationStatus(Quaternion inputRotationStatus)
        {
            throw new NotImplementedException();
        }

        public void SetSleepingFrameCount(int frameCount)
        {
            throw new NotImplementedException();
        }

        public void SetStaticFrictionCoeff(double staticFrictionCoeff)
        {
            throw new NotImplementedException();
        }

        public void SetTorque(Vector3 torque)
        {
            throw new NotImplementedException();
        }

        IShape IMapper.GetShape()
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
