using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject.Joint
{
    public sealed class PistonJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        PistonConstraint pistonConstraint;

        #endregion

        #region Constructor

        public PistonJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPosition,
            Vector3 pistonAxis,
            double restoreCoefficient,
            double springCoefficient)
        {
            pistonConstraint = new PistonConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPosition,
                pistonAxis,
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
            return pistonConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return pistonConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Piston;
        }

        public int GetKeyIndex()
        {
            return pistonConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return pistonConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return pistonConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            pistonConstraint.SetAxis1AngularLimit(angularLimitMin, angularLimitMax);
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            pistonConstraint.SetAxis1Motor(speedValue, forceLimit);
        }

        public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetAxis2Motor(double speedValue, double forceLimit)
        {
            pistonConstraint.SetAxis2Motor(speedValue, forceLimit);
        }

        public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            pistonConstraint.SetLinearLimit(linearLimitMin, linearLimitMax);
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            pistonConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            pistonConstraint.SetSpringCoefficient(springCoefficient);
        }

        #endregion
    }
}
