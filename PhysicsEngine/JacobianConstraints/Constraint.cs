using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;

namespace SharpPhysicsEngine
{
    internal abstract class Constraint: IConstraint, IConstraintBuilder
    {
        #region Fields

        protected Vector3 xVec = new Vector3(1.0, 0.0, 0.0);
        protected Vector3 xVecNeg = new Vector3(-1.0, 0.0, 0.0);
        protected Vector3 yVec = new Vector3(0.0, 1.0, 0.0);
        protected Vector3 yVecNeg = new Vector3(0.0, -1.0, 0.0);
        protected Vector3 zVec = new Vector3(0.0, 0.0, 1.0);
        protected Vector3 zVecNeg = new Vector3(0.0, 0.0, -1.0);

        protected IShape ShapeA;
        protected IShape ShapeB;
        protected int KeyIndex;
        protected double SpringCoefficient;
        protected double RestoreCoefficient;

        #endregion

        #region Constructor

        public Constraint(
            IShape shapeA,
            IShape shapeB,
            double restoreCoefficient,
            double springCoefficient)
        {
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = GetHashCode();
            SpringCoefficient = springCoefficient;
            RestoreCoefficient = restoreCoefficient;
        }
        
        #endregion

        #region Public Methods

        public int GetKeyIndex()
        {
            return KeyIndex;
        }

        public int GetObjectIndexA()
        {
            return ShapeA.ID;
        }

        public int GetObjectIndexB()
        {
            return ShapeB.ID;
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            RestoreCoefficient = restoreCoefficient;
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            SpringCoefficient = springCoefficient;
        }

        public abstract List<JacobianConstraint> BuildJacobian(double? baumStabilization = null);
        public abstract Vector3 GetAnchorPosition();
        public abstract JointType GetJointType();
        public abstract void AddTorque(double torqueAxis1, double torqueAxis2);
        public abstract void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax);
        public abstract void SetAxis1Motor(double speedValue, double forceLimit);
        public abstract void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax);
        public abstract void SetAxis2Motor(double speedValue, double forceLimit);
        public abstract void SetLinearLimit(double linearLimitMin, double linearLimitMax);
              
        #endregion
    }
}
