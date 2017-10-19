using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;

namespace SharpPhysicsEngine
{
    public sealed class SoftBodyConstraint : IConstraint, IConstraintBuilder
    {
        #region Public Fields

        const JointType jointType = JointType.SoftJoint;

        SoftShapePoint PointA;
        SoftShapePoint PointB;
        int KeyIndex;

        readonly ISoftShape Shape;
        readonly double SpringCoefficient;
        readonly Vector3 StartAnchorPoint;

        Vector3 AnchorPoint;
        Vector3 StartErrorAxis1;
        Vector3 StartErrorAxis2;
        Quaternion RelativeOrientation;
        double RestoreCoefficient;

        #endregion

        #region Constructor

        public SoftBodyConstraint(
            SoftShapePoint pointA,
            SoftShapePoint pointB,
            ISoftShape shape,
            double restoreCoefficient,
            double springCoefficient)
        {
            PointA = pointA;
            PointB = pointB;
            KeyIndex = GetHashCode();
            SpringCoefficient = springCoefficient;
            RestoreCoefficient = restoreCoefficient;
            Shape = shape;

            StartAnchorPoint = (PointA.Position - PointB.Position) * 0.5;

            Vector3 relativePos = StartAnchorPoint - PointA.StartPosition;
            
            AnchorPoint = relativePos + PointA.Position;

            StartErrorAxis1 = AnchorPoint - PointA.Position;

            StartErrorAxis2 = AnchorPoint - PointB.Position;

            RelativeOrientation = new Quaternion(1.0, 0.0, 0.0, 0.0);
        }

        #endregion

        #region Public Methods

        #region IConstraintBuilder

        /// <summary>
        /// Builds the fixed joint.
        /// </summary>
        /// <returns>The fixed joint.</returns>
        /// <param name="simulationObjs">Simulation objects.</param>
        public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
        {
            var fixedConstraints = new List<JacobianConstraint>();

            AnchorPoint = (PointA.RotationMatrix *
                                (StartAnchorPoint - PointA.StartPosition)) +
                                PointA.Position;

            #region Init Linear

            Vector3 r1 = PointA.RotationMatrix *
                         StartErrorAxis1;

            Vector3 r2 = PointB.RotationMatrix *
                         StartErrorAxis2;

            Vector3 p1 = PointA.Position + r1;
            Vector3 p2 = PointB.Position + r2;

            Vector3 linearError = p2 - p1;

            Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix();
            Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix();

            #endregion

            #region Init Angular

            Vector3 angularError = JacobianCommon.GetFixedAngularError(
                PointA,
                PointB,
                RelativeOrientation);

            #endregion

            #region Jacobian Constraint

            ConstraintType constraintType = ConstraintType.Joint;

            if (SpringCoefficient > 0)
                constraintType = ConstraintType.SoftJoint;

            double restoreCoeff = (baumStabilization.HasValue) ? baumStabilization.Value : RestoreCoefficient;

            double constraintLimit = restoreCoeff * linearError.x;

            //DOF 1

            fixedConstraints.Add(JacobianCommon.GetDOF(
                new Vector3(1.0, 0.0, 0.0),
                new Vector3(-1.0, 0.0, 0.0),
                new Vector3(-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
                new Vector3(skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                SpringCoefficient,
                0.0,
                constraintType));

            //DOF 2

            constraintLimit = RestoreCoefficient * linearError.y;

            fixedConstraints.Add(JacobianCommon.GetDOF(
                new Vector3(0.0, 1.0, 0.0),
                new Vector3(0.0, -1.0, 0.0),
                new Vector3(-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
                new Vector3(skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                SpringCoefficient,
                0.0,
                constraintType));

            //DOF 3

            constraintLimit = restoreCoeff * linearError.z;

            fixedConstraints.Add(JacobianCommon.GetDOF(
                new Vector3(0.0, 0.0, 1.0),
                new Vector3(0.0, 0.0, -1.0),
                new Vector3(-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
                new Vector3(skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                SpringCoefficient,
                0.0,
                constraintType));

            //DOF 4

            constraintLimit = restoreCoeff * angularError.x;

            fixedConstraints.Add(JacobianCommon.GetDOF(
                new Vector3(0.0, 0.0, 0.0),
                new Vector3(0.0, 0.0, 0.0),
                new Vector3(-1.0, 0.0, 0.0),
                new Vector3(1.0, 0.0, 0.0),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                SpringCoefficient,
                0.0,
                constraintType));

            //DOF 5

            constraintLimit = restoreCoeff * angularError.y;

            fixedConstraints.Add(JacobianCommon.GetDOF(
                new Vector3(0.0, 0.0, 0.0),
                new Vector3(0.0, 0.0, 0.0),
                new Vector3(0.0, -1.0, 0.0),
                new Vector3(0.0, 1.0, 0.0),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                SpringCoefficient,
                0.0,
                constraintType));

            //DOF 6

            constraintLimit = restoreCoeff * angularError.z;

            fixedConstraints.Add(JacobianCommon.GetDOF(
                new Vector3(0.0, 0.0, 0.0),
                new Vector3(0.0, 0.0, 0.0),
                new Vector3(0.0, 0.0, -1.0),
                new Vector3(0.0, 0.0, 1.0),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                SpringCoefficient,
                0.0,
                constraintType));

            #endregion

            return fixedConstraints;
        }

        #endregion

        #region IConstraint

        public JointType GetJointType()
        {
            return jointType;
        }

        public int GetObjectIndexA()
        {
            return PointA.GetID();
        }

        public int GetObjectIndexB()
        {
            return PointB.GetID();
        }

        public int GetKeyIndex()
        {
            return KeyIndex;
        }

        public Vector3 GetStartAnchorPosition()
        {
            return StartAnchorPoint;
        }

        public Vector3 GetAnchorPosition()
        {
            return AnchorPoint;
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            RestoreCoefficient = restoreCoefficient;
        }

        #region NotSupportedMethods

        void IConstraint.SetAxis1Motor(double speedValue, double forceLimit)
        {
            throw new NotSupportedException();
        }

        void IConstraint.SetAxis2Motor(double speedValue, double forceLimit)
        {
            throw new NotSupportedException();
        }

        void IConstraint.SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        void IConstraint.SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        void IConstraint.SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            throw new NotSupportedException();
        }

        void IConstraint.AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotSupportedException();
        }

        #endregion

        #endregion

        #endregion
    }
}
