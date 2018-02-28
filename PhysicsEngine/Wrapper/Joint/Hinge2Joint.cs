using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper.Joint
{
    public sealed class Hinge2Joint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        Hinge2Constraint hinge2Constraint;

        #endregion

        #region Constructor

        public Hinge2Joint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double restoreCoefficient,
            double springCoefficientHingeAxis,
            double springCoefficient)
        {
            hinge2Constraint = new Hinge2Constraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPosition,
                hingeAxis,
                rotationAxis,
                restoreCoefficient,
                springCoefficientHingeAxis,
                springCoefficient);
        }

        public Hinge2Joint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            ICollisionShape externalSyncShape,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double restoreCoefficient,
            double springCoefficientHingeAxis,
            double springCoefficient)
        {
            hinge2Constraint = new Hinge2Constraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                ((IMapper)externalSyncShape).GetShape(),
                startAnchorPosition,
                hingeAxis,
                rotationAxis,
                restoreCoefficient,
                springCoefficientHingeAxis,
                springCoefficient);
        }

        #endregion


        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            hinge2Constraint.AddTorque(torqueAxis1, torqueAxis2);
        }

        public Vector3 GetAnchorPosition()
        {
            return hinge2Constraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return hinge2Constraint;
        }

        public JointType GetJointType()
        {
            return JointType.Hinge2;
        }

        public int GetKeyIndex()
        {
            return hinge2Constraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return hinge2Constraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return hinge2Constraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            hinge2Constraint.SetAxis1AngularLimit(angularLimitMin, angularLimitMax);
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            hinge2Constraint.SetAxis1Motor(speedValue, forceLimit);
        }

        public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            hinge2Constraint.SetAxis2AngularLimit(angularLimitMin, angularLimitMax);
        }

        public void SetAxis2Motor(double speedValue, double forceLimit)
        {
            hinge2Constraint.SetAxis2Motor(speedValue, forceLimit);
        }

        public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            hinge2Constraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            hinge2Constraint.SetSpringCoefficient(springCoefficient);
        }

        public void RotateAxis1(double angle)
        {
            hinge2Constraint.RotateAxis1(angle);
        }
        
        #endregion
    }
}
