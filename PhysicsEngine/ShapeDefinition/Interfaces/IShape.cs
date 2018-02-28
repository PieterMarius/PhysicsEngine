using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal interface IShape : IShapeCommon
    {
        double RestoreCoeff { get; }
        double DynamicFrictionCoeff { get; }
        bool ExcludeFromCollisionDetection { get; }
        double RestitutionCoeff { get; }
        Quaternion RotationStatus { get; }
        int SleepingFrameCount { get; }
        Vector3 StartPosition { get; }
        double StaticFrictionCoeff { get; }
        Vector3 TorqueValue { get; }

        void SetAABB();
        void SetRestoreCoeff(double value);
        void SetDynamicFrictionCoeff(double dynamicFrictionCoeff);
        void SetExcludeFromCollisionDetection(bool excludeFromCollisionDetection);
        void SetRestitutionCoeff(double restitutionCoeff);
        void SetRotationStatus(Quaternion inputRotationStatus);
        void SetSleepingFrameCount(int frameCount);
        void SetStaticFrictionCoeff(double staticFrictionCoeff);
        void SetTorque(Vector3 torque);
        void Rotate(Vector3 versor, double angle);
    }
}