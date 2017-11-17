using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject.Joint
{
    public sealed class FixedJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        FixedJointConstraint fixedJointConstraint;

        #endregion
        
        #region Constructor

        public FixedJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            double restoreCoeff,
            double springCoeff)
        {

            fixedJointConstraint = new FixedJointConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                restoreCoeff,
                springCoeff);
        }
        
        #endregion

        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotSupportedException();
        }

        public Vector3 GetAnchorPosition()
        {
            return fixedJointConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return fixedJointConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Fixed;
        }

        public int GetKeyIndex()
        {
            return fixedJointConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return fixedJointConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return fixedJointConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            throw new NotSupportedException();
        }

        public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetAxis2Motor(double speedValue, double forceLimit)
        {
            throw new NotSupportedException();
        }

        public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            fixedJointConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            fixedJointConstraint.SetSpringCoefficient(springCoefficient);
        }

        #endregion
    }
}
