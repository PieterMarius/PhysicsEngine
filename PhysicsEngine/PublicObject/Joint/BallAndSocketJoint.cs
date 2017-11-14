using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.PublicObject.Joint
{
    public sealed class BallAndSocketJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        BallAndSocketConstraint ballAndSocketConstraint;

        #endregion


        #region Constructor

        public BallAndSocketJoint()
        {

        }


        #endregion



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
            return ballAndSocketConstraint;
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
    }
}
