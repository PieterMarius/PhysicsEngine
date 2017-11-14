using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject.Joint
{
    public sealed class Hinge2Joint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        Hinge2Constraint hinge2Constraint;

        #endregion

        #region Constructor

        public Hinge2Joint()
        {

        }

        #endregion


        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotImplementedException();
        }

        public Vector3 GetAnchorPosition()
        {
            throw new NotImplementedException();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return hinge2Constraint;
        }

        public JointType GetJointType()
        {
            throw new NotImplementedException();
        }

        public int GetKeyIndex()
        {
            throw new NotImplementedException();
        }

        public int GetObjectIndexA()
        {
            throw new NotImplementedException();
        }

        public int GetObjectIndexB()
        {
            throw new NotImplementedException();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotImplementedException();
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            throw new NotImplementedException();
        }

        public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotImplementedException();
        }

        public void SetAxis2Motor(double speedValue, double forceLimit)
        {
            throw new NotImplementedException();
        }

        public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            throw new NotImplementedException();
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            throw new NotImplementedException();
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
