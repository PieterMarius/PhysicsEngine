using System;
using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
    public class SoftShapePoint: IShapeCommon, Identity
    {
        #region Fields

        int Identity.ID { get; set; }
        public Vector3 Position { get; private set; }
        public Vector3 StartPosition { get; private set; }
        //TODO: delete ???
        public Vector3 AngularVelocity { get; private set; }
        public Vector3 LinearVelocity { get; private set; }
        public Matrix3x3 RotationMatrix { get; private set; }
        public Quaternion RotationStatus { get; private set; }
        public double Diameter { get; private set; }
        public Matrix3x3 InertiaTensor { get; private set; }
        public double InverseMass { get; private set; }
        public ObjectType ObjectType { get; private set; }
        public Vector3 TempAngularVelocity { get; private set; }
        public Vector3 TempLinearVelocity { get; private set; }
        
        #endregion

        #region Constructor

        public SoftShapePoint(double diameter)
        {
            Diameter = diameter;
        }

        #endregion

        #region Public Methods

        public int GetID()
        {
            return ((Identity)this).ID;
        }

        public void SetPosition(Vector3 position)
        {
            Position = position;
        }

        public void SetAngularVelocity(Vector3 angularVelocity)
        {
            AngularVelocity = angularVelocity;
        }

        public void SetLinearVelocity(Vector3 linearVelocity)
        {
            LinearVelocity = linearVelocity;
        }

        public void SetTempAngularVelocity(Vector3 angularVelocity)
        {
            TempAngularVelocity = angularVelocity;
        }

        public void SetTempLinearVelocity(Vector3 linearVelocity)
        {
            TempLinearVelocity = linearVelocity;
        }

        #endregion
    }
}
