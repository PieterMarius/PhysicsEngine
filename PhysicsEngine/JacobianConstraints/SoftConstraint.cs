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

using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;

namespace SharpPhysicsEngine
{
    internal sealed class SoftConstraint : IConstraint
    {
        #region Private Fields

        const JointType jointType = JointType.SoftJoint;

        readonly Vector3d xVec = new Vector3d(1.0, 0.0, 0.0);
        readonly Vector3d xVecNeg = new Vector3d(-1.0, 0.0, 0.0);
        readonly Vector3d yVec = new Vector3d(0.0, 1.0, 0.0);
        readonly Vector3d yVecNeg = new Vector3d(0.0, -1.0, 0.0);
        readonly Vector3d zVec = new Vector3d(0.0, 0.0, 1.0);
        readonly Vector3d zVecNeg = new Vector3d(0.0, 0.0, -1.0);

        readonly ISoftShape Shape;
        SoftShapePoint PointA;
        SoftShapePoint PointB;
        int KeyIndex;
        bool ActivateAngularConstraint;
        
        double SpringCoeff;
        double AngularSpringCoeff;
        readonly Vector3d StartAnchorPoint;

        Vector3d AnchorPoint;
        Vector3d StartErrorAxis1;
        Vector3d StartErrorAxis2;
        double ErrorReductionParam;
        double AngularErrorReductionParam;
        Quaternion RelativeOrientation;

        #endregion

        #region Constructor

        public SoftConstraint(
            SoftShapePoint pointA,
            SoftShapePoint pointB,
            ISoftShape shape,
            double errorReductionParam,
            double springCoefficient,
            double angularErrorReductionParam,
            double angularSpringCoeff)
        {
            PointA = pointA;
            PointB = pointB;
            KeyIndex = GetHashCode();
            SpringCoeff = springCoefficient;
            AngularSpringCoeff = angularSpringCoeff;
            ErrorReductionParam = errorReductionParam;
            AngularErrorReductionParam = angularErrorReductionParam;
            Shape = shape;
            ActivateAngularConstraint = false;

            if (angularErrorReductionParam != 0.0 && 
                angularSpringCoeff != 0.0)
                ActivateAngularConstraint = true;
            
            StartAnchorPoint = (PointB.StartPosition + PointA.StartPosition) * 0.5;

            Vector3d relativePos = PointA.RotationMatrix * (StartAnchorPoint - PointA.StartPosition);

            AnchorPoint = relativePos + PointA.Position;

            StartErrorAxis1 = PointA.RotationMatrix.Transpose() *
                                     (AnchorPoint - PointA.Position);

            StartErrorAxis2 = PointB.RotationMatrix.Transpose() *
                                     (AnchorPoint - PointB.Position);

            RelativeOrientation = pointB.RotationStatus.Inverse() *
                                  pointA.RotationStatus;
        }

        public SoftConstraint(
            SoftShapePoint pointA,
            SoftShapePoint pointB,
            ISoftShape shape,
            double errorReductionParam,
            double springCoefficient)
            : this(pointA, pointB, shape, errorReductionParam, springCoefficient, 0.0, 0.0)
        { }

        #endregion

        #region Public Methods

        #region IConstraintBuilder

        /// <summary>
        /// Builds the fixed joint.
        /// </summary>
        /// <returns>The fixed joint.</returns>
        /// <param name="simulationObjs">Simulation objects.</param>
        public List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
        {
            var softConstraints = new List<JacobianConstraint>();

            #region Init Linear

            Vector3d r1 = PointA.RotationMatrix *
                         StartErrorAxis1;

            Vector3d r2 = PointB.RotationMatrix *
                         StartErrorAxis2;

            Vector3d p1 = PointA.Position + r1;
            Vector3d p2 = PointB.Position + r2;

            Vector3d linearError = p2 - p1;

            Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix();
            Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix();

            #endregion

            #region Init Angular

            Vector3d angularError = JacobianCommon.GetFixedAngularError(
                PointA,
                PointB,
                RelativeOrientation);

            #endregion

            #region Jacobian Constraint

            double freq = 1.0 / timeStep;
            double errorReduction = ErrorReductionParam * freq;
            double springCoefficient = SpringCoeff * freq;
            
            ConstraintType constraintType = ConstraintType.SoftJoint;

            double constraintLimit = errorReduction * linearError.x;

            //DOF 1

            softConstraints.Add(JacobianCommon.GetDOF(
                xVec,
                xVecNeg,
                new Vector3d(-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
                new Vector3d(skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                springCoefficient,
                0.0,
                constraintType));

            //DOF 2

            constraintLimit = errorReduction * linearError.y;

            softConstraints.Add(JacobianCommon.GetDOF(
                yVec,
                yVecNeg,
                new Vector3d(-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
                new Vector3d(skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                springCoefficient,
                0.0,
                constraintType));

            //DOF 3

            constraintLimit = errorReduction * linearError.z;

            softConstraints.Add(JacobianCommon.GetDOF(
                zVec,
                zVecNeg,
                new Vector3d(-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
                new Vector3d(skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
                PointA,
                PointB,
                0.0,
                constraintLimit,
                springCoefficient,
                0.0,
                constraintType));

            if (ActivateAngularConstraint)
            {
                double angErrorReduction = AngularErrorReductionParam * freq;
                double angSpringCoefficient = AngularSpringCoeff * freq;

                //DOF 4

                constraintLimit = angErrorReduction * angularError.x;

                softConstraints.Add(JacobianCommon.GetDOF(
                    xVecNeg,
                    xVec,
                    PointA,
                    PointB,
                    0.0,
                    constraintLimit,
                    angSpringCoefficient,
                    0.0,
                    constraintType));

                //DOF 5

                constraintLimit = angErrorReduction * angularError.y;

                softConstraints.Add(JacobianCommon.GetDOF(
                    yVecNeg,
                    yVec,
                    PointA,
                    PointB,
                    0.0,
                    constraintLimit,
                    angSpringCoefficient,
                    0.0,
                    constraintType));

                //DOF 6

                constraintLimit = angErrorReduction * angularError.z;

                softConstraints.Add(JacobianCommon.GetDOF(
                    zVecNeg,
                    zVec,
                    PointA,
                    PointB,
                    0.0,
                    constraintLimit,
                    angSpringCoefficient,
                    0.0,
                    constraintType));
            }

            #endregion

            return softConstraints;
        }

        #endregion

        #region IConstraint

        public JointType GetJointType()
        {
            return jointType;
        }

        public int GetObjectIndexA()
        {
            return PointA.ID;
        }

        public int GetObjectIndexB()
        {
            return PointB.ID;
        }

        public int GetKeyIndex()
        {
            return KeyIndex;
        }

        public Vector3d GetStartAnchorPosition()
        {
            return StartAnchorPoint;
        }

        public Vector3d GetAnchorPosition()
        {
            return (PointA.RotationMatrix *
                   (StartAnchorPoint - PointA.StartPosition)) +
                   PointA.Position;
        }

        public void SetErrorReductionParam(double restoreCoefficient)
        {
            ErrorReductionParam = restoreCoefficient;
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            SpringCoeff = springCoefficient;
        }

        public double GetErrorReductionParam()
        {
            return ErrorReductionParam;
        }

        public double GetSpringCoefficient()
        {
            return SpringCoeff;
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

        void IConstraint.SetAxis1AngularLimit(double? angularLimitMin, double? angularLimitMax)
        {
            throw new NotSupportedException();
        }

        void IConstraint.SetAxis2AngularLimit(double? angularLimitMin, double? angularLimitMax)
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
