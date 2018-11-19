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
    internal sealed class HingeConstraint: Constraint
	{
		#region Private Fields

		const JointType jointType = JointType.Hinge;
        
        readonly Vector3d StartAnchorPoint;
		readonly Vector3d HingeAxis;
		readonly Vector3d StartErrorAxis1;
		readonly Vector3d StartErrorAxis2;
		readonly Quaternion RelativeOrientation;

		double? AngularLimitMin;
		double? AngularLimitMax;
		double? SpeedValue;
		double? ForceLimit;
        		
		Vector3d AnchorPoint;

        #endregion

        #region Constructor

        public HingeConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3d startAnchorPosition,
            Vector3d hingeAxis,
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

            StartErrorAxis2 = shapeB.RotationMatrix.Transpose() *
                                     (AnchorPoint - shapeB.Position);

            RelativeOrientation = shapeB.RotationStatus.Inverse() *
                                        ShapeA.RotationStatus;

            HingeAxis = hingeAxis.Normalize();
        }

		#endregion

		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the hinge joint.
		/// </summary>
		/// <returns>The hinge joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public override List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
		{
			var hingeConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            
            #region Init Linear

			Vector3d axisRotated = GetHingeAxis();

			Vector3d t1 = GeometryUtils.GetPerpendicularVector (axisRotated).Normalize ();
			Vector3d t2 = Vector3d.Cross (axisRotated, t1).Normalize ();

			Vector3d r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3d r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Matrix3x3 skewP1 = Matrix3x3.GetSkewSymmetricMatrix (r1);
			Matrix3x3 skewP2 = Matrix3x3.GetSkewSymmetricMatrix (r2);

			Vector3d p1 = simulationObjectA.Position + r1;
			Vector3d p2 = simulationObjectB.Position + r2;

			Vector3d linearError = p2 - p1;

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

            #region Base Constraint

            ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double constraintLimit = errorReduction * linearError.x;

			hingeConstraints.Add (JacobianCommon.GetDOF(
                xVec,
                xVecNeg,
                new Vector3d (-skewP1.r1c1, -skewP1.r1c2, -skewP1.r1c3),
				new Vector3d (skewP2.r1c1,skewP2.r1c2,skewP2.r1c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = errorReduction * linearError.y;

			hingeConstraints.Add (JacobianCommon.GetDOF (
                yVec,
                yVecNeg,
                new Vector3d (-skewP1.r2c1, -skewP1.r2c2, -skewP1.r2c3),
				new Vector3d (skewP2.r2c1,skewP2.r2c2,skewP2.r2c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = errorReduction * linearError.z;

			hingeConstraints.Add (JacobianCommon.GetDOF (
                zVec,
                zVecNeg,
                new Vector3d (-skewP1.r3c1, -skewP1.r3c2, -skewP1.r3c3),
				new Vector3d (skewP2.r3c1,skewP2.r3c2,skewP2.r3c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 4

			double angularLimit = errorReduction *
				t1.Dot(angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
                    	-1.0 * t1, 
					1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
                    springCoefficient,
					0.0,
					constraintType));

			//DOF 5

			angularLimit = errorReduction *
				t2.Dot(angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
                    	-1.0 * t2, 
					1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					0.0,
					angularLimit,
                    springCoefficient,
					0.0,
					constraintType));

			#endregion

			#region Limit Constraints 

			if (AngularLimitMin.HasValue && 
				AngularLimitMax.HasValue)
			{
				double angle = JacobianCommon.GetAngle (
					simulationObjectA,
					simulationObjectB,
					RelativeOrientation,
					HingeAxis);

				JacobianConstraint? jContact = 
					JacobianCommon.GetAngularLimit (
                        angle,
                        errorReduction,
						0.0,
						simulationObjectA, 
						simulationObjectB, 
						axisRotated,
						AngularLimitMin.Value,
						AngularLimitMax.Value);

				if (jContact != null)
					hingeConstraints.Add (jContact.Value);
			}

			#endregion

			#region Motor Contraint

			if(SpeedValue.HasValue &&
				ForceLimit.HasValue)
			{
				hingeConstraints.Add (
					JacobianCommon.GetDOF (
                        	-1.0 * axisRotated, 
						1.0 * axisRotated, 
						simulationObjectA, 
						simulationObjectB, 
						SpeedValue.Value,
						0.0,
						0.0,
						ForceLimit.Value,
						ConstraintType.JointMotor));
			}

			#endregion

			#endregion

			return hingeConstraints;
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

        public override void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedValue = speedValue;
			ForceLimit = forceLimit;
		}

		public override void SetAxis1AngularLimit(double? angularLimitMin, double? angularLimitMax)
		{
			AngularLimitMin = angularLimitMin;
			AngularLimitMax = angularLimitMax;
		}

		public override void AddTorque(double torqueAxis1, double torqueAxis2)
		{
			Vector3d hingeAxis = ShapeA.RotationMatrix * HingeAxis;

			Vector3d torque = hingeAxis * torqueAxis1;

            ShapeA.SetTorque(ShapeA.TorqueValue + torque);
            ShapeB.SetTorque(ShapeB.TorqueValue - torque);
		}

        public Vector3d GetHingeAxis()
        {
            return ShapeA.RotationMatrix* HingeAxis;
        }
        
        #region NotImplementedMethods

        public override void SetAxis2Motor(double speedValue, double forceLimit)
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
        
		#endregion

		#endregion

		#endregion

	}
}

