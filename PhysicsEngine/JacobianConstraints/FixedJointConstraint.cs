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
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine
{
    internal sealed class FixedJointConstraint: Constraint
	{
		#region Private Fields

		const JointType jointType = JointType.Fixed;
               
		Vector3d StartAnchorPoint;

		Vector3d AnchorPoint;
		Vector3d StartErrorAxis1;
		Vector3d StartErrorAxis2;
		Quaternion RelativeOrientation;

        #endregion

        #region Constructor

        public FixedJointConstraint(
            IShape shapeA,
            IShape shapeB,
            double errorReductionParam,
            double springCoefficient)
            : base(shapeA, shapeB, errorReductionParam, springCoefficient)
        {
            StartAnchorPoint = (shapeB.Position + shapeA.Position) * 0.5;

            Vector3d relativePos = StartAnchorPoint - shapeA.InitCenterOfMass;
            relativePos = shapeA.RotationMatrix * relativePos;

            AnchorPoint = relativePos + shapeA.Position;

            StartErrorAxis1 = shapeA.RotationMatrix.Transpose() *
                                     (AnchorPoint - shapeA.Position);

            StartErrorAxis2 = shapeB.RotationMatrix.Transpose() *
                                     (AnchorPoint - shapeB.Position);

            RelativeOrientation = shapeB.RotationStatus.Inverse() *
                                  shapeA.RotationStatus;
        }
                
        #endregion

        #region Public Methods

        #region IConstraintBuilder

        /// <summary>
        /// Builds the fixed joint.
        /// </summary>
        /// <returns>The fixed joint.</returns>
        /// <param name="simulationObjs">Simulation objects.</param>
        public override List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
		{
			var fixedConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            
            #region Init Linear

			Vector3d r1 = simulationObjectA.RotationMatrix *
						 StartErrorAxis1;

			Vector3d r2 = simulationObjectB.RotationMatrix *
						 StartErrorAxis2;

			Vector3d p1 = simulationObjectA.Position + r1;
			Vector3d p2 = simulationObjectB.Position + r2;

			Vector3d linearError = p2 - p1;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix ();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix ();

			#endregion

			#region Init Angular

			Vector3d angularError = JacobianCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				RelativeOrientation);

            #endregion

            #region Jacobian Constraint

            double freq = 1.0 / timeStep;
            double errorReduction = ErrorReductionParam * freq;
            double springCoefficient = SpringCoefficient * freq;

            ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			double restoreCoeff = baumStabilization ?? errorReduction;

			double constraintLimit = restoreCoeff * linearError.x;

			//DOF 1

			fixedConstraints.Add (JacobianCommon.GetDOF(
                xVec,
                xVecNeg,
                new Vector3d (-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3d (skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = restoreCoeff * linearError.y;

			fixedConstraints.Add (JacobianCommon.GetDOF(
                yVec,
                yVecNeg,
                new Vector3d (-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3d (skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = restoreCoeff * linearError.z;

			fixedConstraints.Add (JacobianCommon.GetDOF (
                zVec,
                zVecNeg,
                new Vector3d (-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3d (skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 4

			constraintLimit = restoreCoeff * 2.0 * angularError.x;

			fixedConstraints.Add (JacobianCommon.GetDOF (
                xVecNeg,
                xVec,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 5

			constraintLimit = restoreCoeff * 2.0 * angularError.y;

			fixedConstraints.Add (JacobianCommon.GetDOF (
                yVecNeg,
                yVec,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 6

			constraintLimit = restoreCoeff * 2.0 * angularError.z;

			fixedConstraints.Add (JacobianCommon.GetDOF (
                zVecNeg,
                zVec,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			#endregion

			return fixedConstraints;
		}

		#endregion

		#region IConstraint

		public override JointType GetJointType()
		{
			return jointType;
		}

        public Vector3d GetStartAnchorPosition()
		{
			return StartAnchorPoint;
		}

		public override Vector3d GetAnchorPosition()
		{
            return (ShapeA.RotationMatrix *
                   (StartAnchorPoint - ShapeA.InitCenterOfMass)) +
                   ShapeA.Position;
        }

        #region NotSupportedMethods

        public override void SetAxis1Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

		public override void SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

        public override void SetAxis1AngularLimit(double? angularLimitMin, double? angularLimitMax)
		{
			throw new NotSupportedException();
		}

        public override void SetAxis2AngularLimit(double? angularLimitMin, double? angularLimitMax)
		{
			throw new NotSupportedException();
		}

        public override void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotSupportedException();
		}

        public override void AddTorque(double torqueAxis1, double torqueAxis2)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion
	}
}

