using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject.Joint
{
    public sealed class SliderJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        SliderConstraint sliderConstraint;

        #endregion

        #region Constructor

        public SliderJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPosition,
            Vector3 sliderAxis,
            double restoreCoefficient,
            double springCoefficient)
        {
            sliderConstraint = new SliderConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPosition,
                sliderAxis,
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
            return sliderConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return sliderConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Slider;
        }

        public int GetKeyIndex()
        {
            return sliderConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return sliderConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return sliderConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            sliderConstraint.SetAxis1Motor(speedValue, forceLimit);
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
            sliderConstraint.SetLinearLimit(linearLimitMin, linearLimitMax);
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            sliderConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            sliderConstraint.SetSpringCoefficient(springCoefficient);
        }

        #endregion
    }
}
