using PhysicsEngineMathUtility;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeDefinition
{
    public interface IShapeCommon
    {
        ObjectType ObjectType { get; }
        Vector3 LinearVelocity { get; }
        Vector3 AngularVelocity { get; }
        Matrix3x3 InertiaTensor { get; }
        double InverseMass { get; }
        Vector3 TempAngularVelocity { get; }
        Vector3 TempLinearVelocity { get; }

        int GetID();
        void SetLinearVelocity(Vector3 inputLinearVelocity);
        void SetAngularVelocity(Vector3 inputAngularVelocity);
        void SetTempAngularVelocity(Vector3 inputAngularVelocity);
        void SetTempLinearVelocity(Vector3 inputLinearVelocity);
    }
}
