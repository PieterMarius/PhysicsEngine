using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ConvexHullWrapper;
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class ConcaveShape : ICollisionShape, IMapper
    {
        #region Fields

        ShapeDefinition.ConcaveShape concaveShape;

        #endregion

        #region Constructor

        public ConcaveShape(
            Vector3[] inputVertexPosition,
            int[][] inputTriangle,
            Vector3 position,
            double mass,
            bool isStatic)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(inputTriangle);

            IConvexHullEngine convexHullEngine = new ConvexHullEngine();

            this.concaveShape = new ShapeDefinition.ConcaveShape(triangleMeshes, inputVertexPosition, convexHullEngine, position, mass, isStatic);
        }

        #endregion

        #region Public Methods

        IShape IMapper.GetShape()
        {
            return concaveShape;
        }

        public ObjectType ObjectType
        {
            get
            {
                return concaveShape.ObjectType;
            }
        }

        public Vector3 Position
        {
            get
            {
                return concaveShape.Position;
            }
        }

        public Vector3 InitCenterOfMass
        {
            get
            {
                return concaveShape.InitCenterOfMass;
            }
        }

        public Vector3 LinearVelocity
        {
            get
            {
                return concaveShape.LinearVelocity;
            }
        }

        public Vector3 AngularVelocity
        {
            get
            {
                return concaveShape.AngularVelocity;
            }
        }

        public Matrix3x3 InertiaTensor
        {
            get
            {
                return concaveShape.InertiaTensor;
            }
        }

        public Matrix3x3 BaseInertiaTensor
        {
            get
            {
                return concaveShape.BaseInertiaTensor;
            }
        }

        public Matrix3x3 RotationMatrix
        {
            get
            {
                return concaveShape.RotationMatrix;
            }
        }

        public double Mass
        {
            get
            {
                return concaveShape.Mass;
            }
        }

        public double InverseMass
        {
            get
            {
                return concaveShape.InverseMass;
            }
        }

        public Vector3 ForceValue
        {
            get
            {
                return concaveShape.ForceValue;
            }
        }

        public bool IsStatic
        {
            get
            {
                return concaveShape.IsStatic;
            }
        }

        public void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            concaveShape.SetExcludeFromCollisionDetection(excludeFromCollisionDetection);
        }

        public Vector3 GetCenterOfMassShiftValue(int index = 0)
        {
            throw new NotImplementedException();
        }

        public int GetGeometryCount()
        {
            throw new NotImplementedException();
        }

        public int GetID()
        {
            return concaveShape.ID;
        }

        public Vector3 GetMaxAABB()
        {
            return CommonUtilities.GetAABBMaxValue(concaveShape.ConvexShapesGeometry);
        }

        public Vector3 GetMinAABB()
        {
            return CommonUtilities.GetAABBMinValue(concaveShape.ConvexShapesGeometry);
        }

        public Vector3[] GetVertices()
        {
            return concaveShape.InputVertexPosition;
        }

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            concaveShape.SetAngularVelocity(inputAngularVelocity);
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            concaveShape.SetBaseInertiaTensor(inputIntertiaTensor);
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            concaveShape.SetDynamicFrictionCoeff(dynamicFrictionCoeff);
        }

        public void SetErrorReductionParam(double value)
        {
            throw new NotImplementedException();
        }

        public void SetForce(Vector3 force)
        {
            concaveShape.SetForce(force);
        }

        public void SetInertiaTensor(Matrix3x3 inertiaTensor)
        {
            concaveShape.SetInertiaTensor(inertiaTensor);
        }

        public void SetIsStatic(bool isStatic)
        {
            concaveShape.SetIsStatic(isStatic);
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            concaveShape.SetLinearVelocity(inputLinearVelocity);
        }

        public void SetMass(double mass)
        {
            concaveShape.SetMass(mass);
        }

        public void SetPosition(Vector3 inputPosition)
        {
            concaveShape.SetPosition(inputPosition);
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            concaveShape.SetRestitutionCoeff(restitutionCoeff);
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            concaveShape.SetRotationMatrix(inputRotationMatrix);
        }

        public void SetRotationStatus(Quaternion inputRotationStatus)
        {
            concaveShape.SetRotationStatus(inputRotationStatus);
        }

        public void SetSleepingFrameCount(int frameCount)
        {
            concaveShape.SetSleepingFrameCount(frameCount);
        }

        public void SetStaticFrictionCoeff(double staticFrictionCoeff)
        {
            concaveShape.SetStaticFrictionCoeff(staticFrictionCoeff);
        }

        public void SetTorque(Vector3 torque)
        {
            concaveShape.SetTorque(torque);
        }

        #endregion
    }
}
