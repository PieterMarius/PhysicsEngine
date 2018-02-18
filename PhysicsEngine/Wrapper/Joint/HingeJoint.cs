using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper.Joint
{
    public sealed class HingeJoint : ICollisionJoint, IMapperJoint
    {

        #region Fields

        HingeConstraint hingeConstraint;

        #endregion

        #region Constructor

        public HingeJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            double restoreCoefficient,
            double springCoefficient)
        {
            hingeConstraint = new HingeConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPosition,
                hingeAxis,
                restoreCoefficient,
                springCoefficient);
        }

        #endregion

        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            hingeConstraint.AddTorque(torqueAxis1, torqueAxis2);
        }

        public Vector3 GetAnchorPosition()
        {
            return hingeConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return hingeConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Hinge;
        }

        public int GetKeyIndex()
        {
            return hingeConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return hingeConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return hingeConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            hingeConstraint.SetAxis1AngularLimit(angularLimitMin, angularLimitMax);
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            hingeConstraint.SetAxis1Motor(speedValue, forceLimit);
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
            hingeConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            hingeConstraint.SetSpringCoefficient(springCoefficient);
        }

        #endregion
    }
}
