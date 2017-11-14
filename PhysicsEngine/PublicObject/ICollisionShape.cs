using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject
{
    public interface ICollisionShape
    {
        ObjectType ObjectType { get; }
        Vector3 Position { get; }
        Vector3 StartPosition { get; }
        Vector3 LinearVelocity { get; }
        Vector3 AngularVelocity { get; }
        Matrix3x3 InertiaTensor { get; }
        Matrix3x3 BaseInertiaTensor { get; }
        Matrix3x3 RotationMatrix { get; }
        double Mass { get; }
        double InverseMass { get; }
        Vector3 ForceValue { get; }

        int GetID();
        void SetGeometry(
            Vector3[] inputVertexPosition,
            TriangleIndexes[] inputTriangle);
        void SetMass(double mass);
        void SetPosition(Vector3 inputPosition);
        void SetLinearVelocity(Vector3 inputLinearVelocity);
        void SetAngularVelocity(Vector3 inputAngularVelocity);
        void SetInertiaTensor(Matrix3x3 inertiaTensor);
        void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor);
        void SetForce(Vector3 force);
        void SetRotationMatrix(Matrix3x3 inputRotationMatrix);
        void SetRestoreCoeff(double value);
        void SetDynamicFrictionCoeff(double dynamicFrictionCoeff);
        void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection);
        void SetRestitutionCoeff(double restitutionCoeff);
        void SetRotationStatus(Quaternion inputRotationStatus);
        void SetSleepingFrameCount(int frameCount);
        void SetStaticFrictionCoeff(double staticFrictionCoeff);
        void SetTorque(Vector3 torque);
    }
}
