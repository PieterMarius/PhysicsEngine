/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using System;
using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
    internal sealed class AngularConstraint : Constraint
    {
        #region Fields

        const JointType jointType = JointType.Angular;

        double SpringCoefficientHingeAxis;
        double SpringCoefficientRotationAxis;
        readonly Vector3d StartAnchorPoint;
        readonly Vector3d StartErrorAxis1;
        readonly Vector3d StartErrorAxis2;
        readonly Quaternion RelativeOrientation1;
        readonly Quaternion RelativeOrientation2;
        readonly Vector3d HingeAxis;
        readonly Vector3d RotationAxis;

        Vector3d AnchorPoint;

        #endregion

        #region Constructor

        public AngularConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3d startAnchorPosition,
            Vector3d hingeAxis,
            Vector3d rotationAxis,
            double errorReductionParam,
            double springCoefficientHingeAxis,
            double springCoefficientRotationAxis)
            : base(shapeA, shapeB, errorReductionParam, 0.0)
        {
            SpringCoefficientHingeAxis = springCoefficientHingeAxis;
            SpringCoefficientRotationAxis = springCoefficientRotationAxis;
            StartAnchorPoint = startAnchorPosition;
            HingeAxis = hingeAxis.Normalize();
            RotationAxis = rotationAxis.Normalize();

            Vector3d relativePos = startAnchorPosition - ShapeA.InitCenterOfMass;
            relativePos = ShapeA.RotationMatrix * relativePos;

            AnchorPoint = relativePos + ShapeA.Position;

            StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeA.Position);

            StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeB.Position);

            Vector3d rHingeAxis = ShapeA.RotationMatrix * HingeAxis;
            Vector3d rRotationAxis = ShapeB.RotationMatrix * RotationAxis;

            RelativeOrientation1 = CalculateRelativeOrientation(
                rHingeAxis,
                rRotationAxis,
                ShapeA.RotationStatus);

            RelativeOrientation2 = CalculateRelativeOrientation(
                rRotationAxis,
                rHingeAxis,
                ShapeB.RotationStatus);
        }

        #endregion
        
        #region Public Methods

        #region IConstraintBuilder
        public override List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
        {
            var angularConstraints = new List<JacobianConstraint>();

            IShape simulationObjectA = ShapeA;
            IShape simulationObjectB = ShapeB;

            double freq = 1.0 / timeStep;
            double errorReduction = ErrorReductionParam * freq;
            double springCoefficientHingeAxis = SpringCoefficientHingeAxis * freq;
            double springCoefficientRotationAxis = SpringCoefficientRotationAxis * freq;

            #region Init Angular

            Vector3d hingeAxis = GetHingeAxis();
            Vector3d rotationAxis = GetRotationAxis();

            double k = hingeAxis.Dot(rotationAxis);
            Vector3d tempPerpendicular = rotationAxis - k * hingeAxis;
            Vector3d t1 = hingeAxis.Cross(tempPerpendicular).Normalize();

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

            double angularLimit = errorReduction * hingeAngle;

            angularConstraints.Add(JacobianCommon.GetDOF(
                hingeAxis,
                -1.0 * hingeAxis,
                simulationObjectA,
                simulationObjectB,
                0.0,
                angularLimit, 
                springCoefficientHingeAxis,
                0.0,
                ConstraintType.Joint));

            angularLimit = errorReduction * twistAngle;

            angularConstraints.Add(JacobianCommon.GetDOF(
                rotationAxis,
                -1.0 * rotationAxis,
                simulationObjectA,
                simulationObjectB,
                0.0,
                angularLimit,
                springCoefficientRotationAxis,
                0.0,
                ConstraintType.Joint));
            
            #endregion

            return angularConstraints;
        }

        #endregion

        #region IConstraint

        public override Vector3d GetAnchorPosition()
        {
            return (ShapeA.RotationMatrix *
                   (StartAnchorPoint - ShapeA.InitCenterOfMass)) +
                   ShapeA.Position;
        }

        public override JointType GetJointType()
        {
            return jointType;
        }

        public Vector3d GetHingeAxis()
        {
            return ShapeA.RotationMatrix * HingeAxis;
        }

        public Vector3d GetRotationAxis()
        {
            return ShapeB.RotationMatrix * RotationAxis;
        }

        public override void AddTorque(
            double torqueAxis1, 
            double torqueAxis2)
        {
            throw new NotImplementedException();
        }
                       
        public override void SetAxis1AngularLimit(double? angularLimitMin, double? angularLimitMax)
        {
            throw new NotImplementedException();
        }

        public override void SetAxis1Motor(double speedValue, double forceLimit)
        {
            throw new NotImplementedException();
        }

        public override void SetAxis2AngularLimit(double? angularLimitMin, double? angularLimitMax)
        {
            throw new NotImplementedException();
        }

        public override void SetAxis2Motor(double speedValue, double forceLimit)
        {
            throw new NotImplementedException();
        }

        public override void SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            throw new NotImplementedException();
        }
        
        #endregion

        #endregion

        #region Private Methods

        double GetAngle2(
            Vector3d axis1,
            Vector3d axis2,
            Vector3d startAxis,
            Quaternion rotationStatus,
            Quaternion startRelativeRotation)
        {
            return -GetAngle1(axis2, axis1, startAxis, rotationStatus, startRelativeRotation);
        }

        double GetAngle1(
            Vector3d axis1,
            Vector3d axis2,
            Vector3d startAxis,
            Quaternion rotationStatus,
            Quaternion startRelativeRotation)
        {
            Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
            Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

            Quaternion mult1 = Quaternion.Multiply1(rotationStatus, rotationQ);
            Quaternion mult2 = Quaternion.Multiply2(mult1, startRelativeRotation);

            var quaternionVectorPart = new Vector3d(
                mult2.b,
                mult2.c,
                mult2.d);

            return JacobianCommon.GetRotationAngle(quaternionVectorPart, mult2.a, startAxis);
        }

        Quaternion CalculateRelativeOrientation(
            Vector3d axis1,
            Vector3d axis2,
            Quaternion bodyRotationStatus)
        {
            Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
            Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

            return Quaternion.Multiply1(bodyRotationStatus, rotationQ);
        }
        
        #endregion
    }

}
