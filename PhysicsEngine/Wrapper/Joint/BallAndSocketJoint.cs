using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper.Joint
{
    public sealed class BallAndSocketJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        BallAndSocketConstraint ballAndSocketConstraint;

        #endregion

        #region Constructor

        public BallAndSocketJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPos,
            double restoreCoeff,
            double springCoeff)
        {
            ballAndSocketConstraint = new BallAndSocketConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPos,
                restoreCoeff,
                springCoeff);
        }
        
        #endregion
        
        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotSupportedException();
        }

        public Vector3 GetAnchorPosition()
        {
            return ballAndSocketConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return ballAndSocketConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.BallAndSocket;
        }

        public int GetKeyIndex()
        {
            return ballAndSocketConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return ballAndSocketConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return ballAndSocketConstraint.GetObjectIndexB();
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
            ballAndSocketConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            ballAndSocketConstraint.SetSpringCoefficient(springCoefficient);
        }
    }
}
