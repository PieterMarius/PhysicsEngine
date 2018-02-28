﻿/******************************************************************************
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
    internal sealed class FixedJointConstraint: IConstraint, IConstraintBuilder
	{
		#region Private Fields

		const JointType jointType = JointType.Fixed;

        readonly Vector3 xVec = new Vector3(1.0, 0.0, 0.0);
        readonly Vector3 xVecNeg = new Vector3(-1.0, 0.0, 0.0);
        readonly Vector3 yVec = new Vector3(0.0, 1.0, 0.0);
        readonly Vector3 yVecNeg = new Vector3(0.0, -1.0, 0.0);
        readonly Vector3 zVec = new Vector3(0.0, 0.0, 1.0);
        readonly Vector3 zVecNeg = new Vector3(0.0, 0.0, -1.0);

        IShape ShapeA;
        IShape ShapeB;
        int KeyIndex;
		double SpringCoefficient;
		Vector3 StartAnchorPoint;

		Vector3 AnchorPoint;
		Vector3 StartErrorAxis1;
		Vector3 StartErrorAxis2;
		Quaternion RelativeOrientation;
		double RestoreCoefficient;

		#endregion

		#region Constructor

		public FixedJointConstraint(
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

			StartAnchorPoint = (shapeB.Position - shapeA.Position) * 0.5;

			Vector3 relativePos = StartAnchorPoint - shapeA.StartPosition;
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
        public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
		{
			var fixedConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            
            #region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
						 StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
						 StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix ();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix ();

			#endregion

			#region Init Angular

			Vector3 angularError = JacobianCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				RelativeOrientation);

			#endregion

			#region Jacobian Constraint

			ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			double restoreCoeff = baumStabilization ?? RestoreCoefficient;

			double constraintLimit = restoreCoeff * linearError.x;

			//DOF 1

			fixedConstraints.Add (JacobianCommon.GetDOF(
                xVec,
                xVecNeg,
                new Vector3 (-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3 (skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = restoreCoeff * linearError.y;

			fixedConstraints.Add (JacobianCommon.GetDOF(
                yVec,
                yVecNeg,
                new Vector3 (-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3 (skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = restoreCoeff * linearError.z;

			fixedConstraints.Add (JacobianCommon.GetDOF (
                zVec,
                zVecNeg,
                new Vector3 (-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3 (skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
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
				SpringCoefficient,
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
				SpringCoefficient,
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
			return ShapeA.ID;
		}

		public int GetObjectIndexB()
		{
			return ShapeB.ID;
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
            return (ShapeA.RotationMatrix *
                   (StartAnchorPoint - ShapeA.StartPosition)) +
                   ShapeA.Position;
        }

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

        public void SetSpringCoefficient(double springCoefficient)
        {
            SpringCoefficient = springCoefficient;
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

