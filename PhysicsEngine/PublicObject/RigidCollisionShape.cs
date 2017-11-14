using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine.PublicObject
{
    public sealed class RigidCollisionShape: ICollisionShape, IMapper
    {
        #region Fileds

        ConvexShape convexShape;

        #endregion

        #region Constructor

        public RigidCollisionShape()
        {
            convexShape = new ConvexShape(ObjectType.RigidBody);
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
        
        #endregion
    }
}
