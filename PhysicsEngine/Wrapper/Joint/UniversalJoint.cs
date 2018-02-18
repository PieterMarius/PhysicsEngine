using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper.Joint
{
    public sealed class UniversalJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        UniversalConstraint universalConstraint;

        #endregion

        #region Constructor

        public UniversalJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double restoreCoefficient,
            double springCoefficient)
        {
            universalConstraint = new UniversalConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPosition,
                hingeAxis,
                rotationAxis,
                restoreCoefficient,
                springCoefficient);
        }

        #endregion
        
        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotSupportedException();
        }

        public Vector3 GetAnchorPosition()
        {
            return universalConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return universalConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Universal;
        }

        public int GetKeyIndex()
        {
            return universalConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return universalConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return universalConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            universalConstraint.SetAxis1AngularLimit(angularLimitMin, angularLimitMax);
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            universalConstraint.SetAxis1Motor(speedValue, forceLimit);
        }

        public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            universalConstraint.SetAxis2AngularLimit(angularLimitMin, angularLimitMax);
        }

        public void SetAxis2Motor(double speedValue, double forceLimit)
        {
            universalConstraint.SetAxis2Motor(speedValue, forceLimit);
        }

        public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            universalConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            universalConstraint.SetSpringCoefficient(springCoefficient);
        }

        #endregion
    }
}
