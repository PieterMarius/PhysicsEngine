using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public interface IShapeCommon
    {
        int ID { get; }
        ObjectType ObjectType { get; }
        Vector3 Position { get; }
        Vector3 LinearVelocity { get; }
        Vector3 AngularVelocity { get; }
        Matrix3x3 InertiaTensor { get; }
        Matrix3x3 BaseInertiaTensor { get; }
        Matrix3x3 RotationMatrix { get; }
        double Mass { get; }
        double InverseMass { get; }
        Vector3 TempAngularVelocity { get; }
        Vector3 TempLinearVelocity { get; }
        Vector3 ForceValue { get; }

        //int GetID();
        void SetMass(double mass);
        void SetPosition(Vector3 inputPosition);
        void SetLinearVelocity(Vector3 inputLinearVelocity);
        void SetAngularVelocity(Vector3 inputAngularVelocity);
        void SetInertiaTensor(Matrix3x3 inertiaTensor);
        void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor);
        void SetForce(Vector3 force);
        void SetTempAngularVelocity(Vector3 inputAngularVelocity);
        void SetTempLinearVelocity(Vector3 inputLinearVelocity);
        void SetRotationMatrix(Matrix3x3 inputRotationMatrix);
    }
}
