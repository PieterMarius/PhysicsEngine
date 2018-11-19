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
    internal sealed class BallAndSocketConstraint : Constraint
	{
		#region Fields

		private const JointType jointType = JointType.BallAndSocket;

        private readonly Vector3d StartAnchorPoint;
        private readonly Vector3d StartErrorAxis1;
        private readonly Vector3d StartErrorAxis2;

        private readonly Vector3d AnchorPoint;

        #endregion

        #region Constructor

        public BallAndSocketConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3d startAnchorPosition,
            double errorReductionParam,
            double springCoefficient)
            : base(shapeA, shapeB, errorReductionParam, springCoefficient)
        {
            StartAnchorPoint = startAnchorPosition;

            Vector3d relativePos = startAnchorPosition - ShapeA.InitCenterOfMass;
            relativePos = ShapeA.RotationMatrix * relativePos;

            AnchorPoint = relativePos + ShapeA.Position;

            StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeA.Position);

            StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeB.Position);
        }

		#endregion

		#region Public Methods

		#region IConstraintBuider

		/// <summary>
		/// Builds the ball socket joint.
		/// </summary>
		/// <returns>The ball socket joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public override List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
		{
			var ballSocketConstraints = new List<JacobianConstraint>();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            
            #region Init Linear

			Vector3d r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3d r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix();

			Vector3d p1 = simulationObjectA.Position + r1;
			Vector3d p2 = simulationObjectB.Position + r2;

			Vector3d linearError = p2 - p1;

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

            ballSocketConstraints.Add(JacobianCommon.GetDOF(
                xVec,
                xVecNeg,
                new Vector3d(-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3d(skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = restoreCoeff * linearError.y;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
                yVec,
                yVecNeg,
                new Vector3d(-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3d(skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = restoreCoeff * linearError.z;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
                zVec,
                zVecNeg,
                new Vector3d(-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3d(skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			#endregion

			return ballSocketConstraints;
		}

		#endregion

		#region IConstraint

		public override JointType GetJointType()
		{
			return jointType;
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

