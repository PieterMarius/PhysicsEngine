using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
    public sealed class AngularConstraint : IConstraint, IConstraintBuilder
    {

        #region Fields

        const JointType jointType = JointType.Angular;

        int IndexA;
        int IndexB;
        int KeyIndex;
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
            SimulationObject[] simulationObject,
            Vector3 startAnchorPosition,
            Vector3 hingeAxis,
            Vector3 rotationAxis,
            double restoreCoefficient)
        {
            IndexA = indexA;
            IndexB = indexB;
            KeyIndex = GetHashCode();
            RestoreCoefficient = restoreCoefficient;
            StartAnchorPoint = startAnchorPosition;
            HingeAxis = hingeAxis.Normalize();
            RotationAxis = rotationAxis.Normalize();

            SimulationObject objectA = simulationObject[IndexA];
            SimulationObject objectB = simulationObject[IndexB];

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
            SimulationObject[] simulationObjs, 
            double? baumStabilization = null)
        {
            var angularConstraints = new List<JacobianContact>();

            SimulationObject simulationObjectA = simulationObjs[IndexA];
            SimulationObject simulationObjectB = simulationObjs[IndexB];

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

            Console.WriteLine("Angle " + hingeAngle + " " + twistAngle);

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
                1.0,
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
                1.0,
                0.0,
                ConstraintType.Joint));

            //JacobianContact? jContact = (JacobianCommon.GetAngularLimit(
            //            IndexA,
            //            IndexB,
            //            hingeAngle,
            //            RestoreCoefficient,
            //            0.0,
            //            simulationObjectA,
            //            simulationObjectB,
            //            hingeAxis,
            //            0.0,
            //            0.0));

            //if (jContact.HasValue)
            //    angularConstraints.Add(jContact.Value);

            //jContact = (JacobianCommon.GetAngularLimit(
            //            IndexA,
            //            IndexB,
            //            twistAngle,
            //            RestoreCoefficient,
            //            0.0,
            //            simulationObjectA,
            //            simulationObjectB,
            //            rotationAxis,
            //            0.0,
            //            0.0));

            //if (jContact.HasValue)
            //    angularConstraints.Add(jContact.Value);

            #endregion

            return angularConstraints;
        }

        #endregion

        #region IConstraint

        public void AddTorque(
            SimulationObject[] objects, 
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
