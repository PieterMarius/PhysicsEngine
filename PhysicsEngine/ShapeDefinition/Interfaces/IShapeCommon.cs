using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
    public interface IShapeCommon
    {
        ObjectType ObjectType { get; }
        Vector3 LinearVelocity { get; }
        Vector3 AngularVelocity { get; }
        Matrix3x3 InertiaTensor { get; }
        Matrix3x3 BaseInertiaTensor { get; }
        double Mass { get; }
        double InverseMass { get; }
        Vector3 TempAngularVelocity { get; }
        Vector3 TempLinearVelocity { get; }

        int GetID();
        void SetMass(double mass);
        void SetLinearVelocity(Vector3 inputLinearVelocity);
        void SetAngularVelocity(Vector3 inputAngularVelocity);
        void SetInertiaTensor(Matrix3x3 inertiaTensor);
        void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor);
        void SetTempAngularVelocity(Vector3 inputAngularVelocity);
        void SetTempLinearVelocity(Vector3 inputLinearVelocity);
    }
}
