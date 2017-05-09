using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
    public interface IShape : IShapeCommon
    {
        double RestoreCoeff { get; }
        double DynamicFrictionCoeff { get; }
        bool ExcludeFromCollisionDetection { get; }
        Vector3 ForceValue { get; }
        Vector3 Position { get; }
        double RestitutionCoeff { get; }
        Matrix3x3 RotationMatrix { get; }
        Quaternion RotationStatus { get; }
        int SleepingFrameCount { get; }
        Vector3 StartPosition { get; }
        double StaticFrictionCoeff { get; }
        Vector3 TorqueValue { get; }

        void SetAABB();
        void SetRestoreCoeff(double value);
        void SetDynamicFrictionCoeff(double dynamicFrictionCoeff);
        void SetExcludeFromCollisionDetection(bool excludeFromCollisionDetection);
        void SetForce(Vector3 force);
        void SetPosition(Vector3 inputPosition);
        void SetRestitutionCoeff(double restitutionCoeff);
        void SetRotationMatrix(Matrix3x3 inputRotationMatrix);
        void SetRotationStatus(Quaternion inputRotationStatus);
        void SetSleepingFrameCount(int frameCount);
        void SetStaticFrictionCoeff(double staticFrictionCoeff);
        void SetTorque(Vector3 torque);
    }
}