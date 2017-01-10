using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
    public interface IShape
    {
        ObjectType ObjectType { get; }
        Vector3 AngularVelocity { get; }
        Matrix3x3 BaseInertiaTensor { get; }
        double BaumgarteStabilizationCoeff { get; }
        int CompoundingConvexObjectCount { get; }
        double DynamicFrictionCoeff { get; }
        bool ExcludeFromCollisionDetection { get; }
        Vector3 ForceValue { get; }
        Matrix3x3 InertiaTensor { get; }
        double InverseMass { get; }
        Vector3 LinearVelocity { get; }
        double Mass { get; }
        IGeometry[] ObjectGeometry { get; }
        double[] PartialMass { get; }
        Vector3 Position { get; }
        double RestitutionCoeff { get; }
        Matrix3x3 RotationMatrix { get; }
        Quaternion RotationStatus { get; }
        int SleepingFrameCount { get; }
        Vector3[] StartCompositePositionObjects { get; }
        Vector3 StartPosition { get; }
        double StaticFrictionCoeff { get; }
        Vector3 TempAngularVelocity { get; }
        Vector3 TempLinearVelocity { get; }
        Vector3 TorqueValue { get; }

        void SetAABB();
        void SetAngularVelocity(Vector3 inputAngularVelocity);
        void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor);
        void SetBaumgarteStabilizationCoeff(double value);
        void SetDynamicFrictionCoeff(double dynamicFrictionCoeff);
        void SetExcludeFromCollisionDetection(bool excludeFromCollisionDetection);
        void SetForce(Vector3 force);
        void SetInertiaTensor(Matrix3x3 inertiaTensor);
        void SetLinearVelocity(Vector3 inputLinearVelocity);
        void SetPosition(Vector3 inputPosition);
        void SetRestitutionCoeff(double restitutionCoeff);
        void SetRotationMatrix(Matrix3x3 inputRotationMatrix);
        void SetRotationStatus(Quaternion inputRotationStatus);
        void SetSleepingFrameCount(int frameCount);
        void SetStaticFrictionCoeff(double staticFrictionCoeff);
        void SetTempAngularVelocity(Vector3 inputAngularVelocity);
        void SetTempLinearVelocity(Vector3 inputLinearVelocity);
        void SetTorque(Vector3 torque);
        void SetObjectGeometry(IGeometry[] geometry);
    }
}