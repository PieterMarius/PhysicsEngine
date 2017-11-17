using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject.Joint
{
    public sealed class AngularJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        AngularConstraint angularConstraint;

        #endregion

        #region Constructor

        public AngularJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPos,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double restoreCoefficient,
            double springCoefficientHingeAxis,
            double springCoefficientRotationAxis)
        {
            angularConstraint = new AngularConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPos,
                hingeAxis,
                rotationAxis,
                restoreCoefficient,
                springCoefficientHingeAxis,
                springCoefficientRotationAxis);
        }
        
        #endregion
        
        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotSupportedException();
        }

        public Vector3 GetAnchorPosition()
        {
            return angularConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return angularConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Angular;
        }

        public int GetKeyIndex()
        {
            return angularConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return angularConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return angularConstraint.GetObjectIndexB();
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
            angularConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            throw new NotSupportedException();
        }

        #endregion
    }
}
