using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using ShapeDefinition;

namespace MonoPhysicsEngine
{
    public sealed class AngularConstraint : IConstraint, IConstraintBuilder
    {

        #region Fields

        const JointType jointType = JointType.Angular;

        int IndexA;
        int IndexB;
        int KeyIndex;
        double SpringCoefficientHingeAxis;
        double SpringCoefficientRotationAxis;
        readonly Vector3 StartAnchorPoint;
        readonly Vector3 StartErrorAxis1;
        readonly Vector3 StartErrorAxis2;
        readonly Quaternion RelativeOrientation1;
        readonly Quaternion RelativeOrientation2;
        readonly Vector3 HingeAxis;
        readonly Vector3 RotationAxis;

        Vector3 AnchorPoint;
        double RestoreCoefficient;


        #endregion

        #region Constructor

        public AngularConstraint(
            int indexA,
            int indexB,
            IShape[] simulationObject,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double restoreCoefficient,
            double springCoefficientHingeAxis,
            double springCoefficientRotationAxis)
        {
            IndexA = indexA;
            IndexB = indexB;
            KeyIndex = GetHashCode();
            RestoreCoefficient = restoreCoefficient;
            SpringCoefficientHingeAxis = springCoefficientHingeAxis;
            SpringCoefficientRotationAxis = springCoefficientRotationAxis;
            StartAnchorPoint = startAnchorPosition;
            HingeAxis = hingeAxis.Normalize();
            RotationAxis = rotationAxis.Normalize();

            IShape objectA = simulationObject[IndexA];
            IShape objectB = simulationObject[IndexB];

            Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
            relativePos = objectA.RotationMatrix * relativePos;

            AnchorPoint = relativePos + objectA.Position;

            StartErrorAxis1 = objectA.RotationMatrix.Transpose() *
                                     (AnchorPoint - objectA.Position);

            StartErrorAxis2 = objectB.RotationMatrix.Transpose() *
                                     (AnchorPoint - objectB.Position);

            Vector3 rHingeAxis = objectA.RotationMatrix * HingeAxis;
            Vector3 rRotationAxis = objectB.RotationMatrix * RotationAxis;

            RelativeOrientation1 = CalculateRelativeOrientation(
                rHingeAxis,
                rRotationAxis,
                objectA.RotationStatus);

            RelativeOrientation2 = CalculateRelativeOrientation(
                rRotationAxis,
                rHingeAxis,
                objectB.RotationStatus);
        }

        #endregion
        
        #region Public Methods

        #region IConstraintBuilder
        public List<JacobianContact> BuildJacobian(
            IShape[] simulationObjs, 
            double? baumStabilization = null)
        {
            var angularConstraints = new List<JacobianContact>();

            IShape simulationObjectA = simulationObjs[IndexA];
            IShape simulationObjectB = simulationObjs[IndexB];

            AnchorPoint = (simulationObjectA.RotationMatrix *
                          (StartAnchorPoint - simulationObjectA.StartPosition)) +
                          simulationObjectA.Position;

            #region Init Angular

            Vector3 hingeAxis = simulationObjectA.RotationMatrix * HingeAxis;
            Vector3 rotationAxis = simulationObjectB.RotationMatrix * RotationAxis;

            double k = hingeAxis.Dot(rotationAxis);
            Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
            Vector3 t1 = hingeAxis.Cross(tempPerpendicular).Normalize();

            double hingeAngle = GetAngle1(
                    hingeAxis,
                    rotationAxis,
                    HingeAxis,
                    simulationObjectA.RotationStatus,
                    RelativeOrientation1);

            double twistAngle = GetAngle2(
                    hingeAxis,
                    rotationAxis,
                    RotationAxis,
                    simulationObjectB.RotationStatus,
                    RelativeOrientation2);
            
            #endregion

            #region Jacobian Constraint

            double angularLimit = RestoreCoefficient * hingeAngle;

            angularConstraints.Add(JacobianCommon.GetDOF(
                IndexA,
                IndexB,
                new Vector3(),
                new Vector3(),
                hingeAxis,
                -1.0 * hingeAxis,
                simulationObjectA,
                simulationObjectB,
                0.0,
                angularLimit,
                SpringCoefficientHingeAxis,
                0.0,
                ConstraintType.Joint));

            angularLimit = RestoreCoefficient * twistAngle;

            angularConstraints.Add(JacobianCommon.GetDOF(
                IndexA,
                IndexB,
                new Vector3(),
                new Vector3(),
                rotationAxis,
                -1.0 * rotationAxis,
                simulationObjectA,
                simulationObjectB,
                0.0,
                angularLimit,
                SpringCoefficientRotationAxis,
                0.0,
                ConstraintType.Joint));
            
            #endregion

            return angularConstraints;
        }

        #endregion

        #region IConstraint

        public void AddTorque(
            ConvexShape[] objects, 
            double torqueAxis1, 
            double torqueAxis2)
        {
            throw new NotImplementedException();
        }

        public Vector3 GetAnchorPosition()
        {
            return AnchorPoint;
        }

        public JointType GetJointType()
        {
            return jointType;
        }

        public int GetKeyIndex()
        {
            return KeyIndex;
        }

        public int GetObjectIndexA()
        {
            return IndexA;
        }

        public int GetObjectIndexB()
        {
            return IndexB;
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

        public void SetObjectIndexA(int index)
        {
            IndexA = index;
        }

        public void SetObjectIndexB(int index)
        {
            IndexB = index;
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            RestoreCoefficient = restoreCoefficient;
        }

        #endregion

        #endregion

        #region Private Methods

        double GetAngle2(
            Vector3 axis1,
            Vector3 axis2,
            Vector3 startAxis,
            Quaternion rotationStatus,
            Quaternion startRelativeRotation)
        {
            return -GetAngle1(axis2, axis1, startAxis, rotationStatus, startRelativeRotation);
        }

        double GetAngle1(
            Vector3 axis1,
            Vector3 axis2,
            Vector3 startAxis,
            Quaternion rotationStatus,
            Quaternion startRelativeRotation)
        {
            Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
            Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

            Quaternion mult1 = Quaternion.Multiply1(rotationStatus, rotationQ);
            Quaternion mult2 = Quaternion.Multiply2(mult1, startRelativeRotation);

            var quaternionVectorPart = new Vector3(
                mult2.b,
                mult2.c,
                mult2.d);

            return JacobianCommon.GetRotationAngle(quaternionVectorPart, mult2.a, startAxis);
        }

        Quaternion CalculateRelativeOrientation(
            Vector3 axis1,
            Vector3 axis2,
            Quaternion bodyRotationStatus)
        {
            Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
            Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

            return Quaternion.Multiply1(bodyRotationStatus, rotationQ);
        }

        #endregion
    }

}
