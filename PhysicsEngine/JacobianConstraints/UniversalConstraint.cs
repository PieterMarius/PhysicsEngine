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
    internal sealed class UniversalConstraint: Constraint 
	{
		#region Public Fields

		const JointType jointType = JointType.Universal;
                
		readonly Vector3d StartAnchorPoint;
		readonly Vector3d HingeAxis;
		readonly Vector3d RotationAxis;
		readonly Vector3d StartErrorAxis1;
		readonly Vector3d StartErrorAxis2;
		readonly Quaternion RelativeOrientation1;
		readonly Quaternion RelativeOrientation2;

		double? AngularLimitMin1;
		double? AngularLimitMax1;
		double? AngularLimitMin2;
		double? AngularLimitMax2;

		double? SpeedHingeAxisLimit;
		double? ForceHingeAxisLimit;
		double? SpeedRotationAxisLimit;
		double? ForceRotationAxisLimit;
		Vector3d AnchorPoint;
		
		#endregion

		#region Constructor

		public UniversalConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3d startAnchorPosition,
			Vector3d hingeAxis,
			Vector3d rotationAxis,
			double erroReductionParam,
			double springCoefficient)
            : base(shapeA, shapeB, erroReductionParam, springCoefficient)
        {
            	StartAnchorPoint = startAnchorPosition;
			HingeAxis = hingeAxis.Normalize ();
			RotationAxis = rotationAxis.Normalize ();

			Vector3d relativePos = startAnchorPosition - ShapeA.InitCenterOfMass;
			relativePos = ShapeA.RotationMatrix * relativePos;

			AnchorPoint = relativePos + ShapeA.Position;

			StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeA.Position);

			StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeB.Position);

			Vector3d rHingeAxis = ShapeA.RotationMatrix * HingeAxis;
			Vector3d rRotationAxis = ShapeB.RotationMatrix * RotationAxis;

			RelativeOrientation1 = CalculateRelativeOrientation (
				rHingeAxis,
				rRotationAxis,
                ShapeA.RotationStatus);

			RelativeOrientation2 = CalculateRelativeOrientation (
				rRotationAxis,
				rHingeAxis,
                ShapeB.RotationStatus);
		}

		#endregion


		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the Universal joint.
		/// </summary>
		/// <returns>The Universal joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public override List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
		{
			var universalConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;

			AnchorPoint = (simulationObjectA.RotationMatrix *
						  (StartAnchorPoint - simulationObjectA.InitCenterOfMass)) +
						  simulationObjectA.Position;

			#region Init Linear

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

			Vector3d hingeAxis = simulationObjectA.RotationMatrix * HingeAxis;
			Vector3d rotationAxis = simulationObjectB.RotationMatrix * RotationAxis;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3d tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3d t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

            #endregion

            #region Jacobian Constraint

            #region Base Constraint

            double freq = 1.0 / timeStep;
            double errorReduction = ErrorReductionParam * freq;
            double springCoefficient = SpringCoefficient * freq;

            ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double constraintLimit = errorReduction * linearError.x;

			universalConstraints.Add (JacobianCommon.GetDOF(
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

			universalConstraints.Add (JacobianCommon.GetDOF (
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

			universalConstraints.Add (JacobianCommon.GetDOF (
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

			double angularLimit = errorReduction * (-k);

			universalConstraints.Add (
				JacobianCommon.GetDOF (
					t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
                    springCoefficient,
					0.0,
					constraintType));

			#endregion

			#region Limit Constraints 

			universalConstraints.AddRange(GetAngularLimit(
				simulationObjectA,
				simulationObjectB,
				hingeAxis,
				rotationAxis,
                errorReduction));

			#endregion

			#endregion

			return universalConstraints;
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

		public override void SetAxis1AngularLimit(double? angularLimitMin, double? angularLimitMax)
		{
			AngularLimitMin1 = angularLimitMin;
			AngularLimitMax1 = angularLimitMax;
		}

		public override void SetAxis2AngularLimit(double? angularLimitMin, double? angularLimitMax)
		{
			AngularLimitMin2 = angularLimitMin;
			AngularLimitMax2 = angularLimitMax;
		}

		public override void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedHingeAxisLimit = speedValue;
			ForceHingeAxisLimit = forceLimit;
		}

		public override void SetAxis2Motor(double speedValue, double forceLimit)
		{
			SpeedRotationAxisLimit = speedValue;
			ForceRotationAxisLimit = forceLimit;
		}
        		
		public override void AddTorque( 
			double torqueAxis1, 
			double torqueAxis2)
		{
			Vector3d hingeAxis = ShapeA.RotationMatrix * HingeAxis;
			Vector3d rotationAxis = ShapeB.RotationMatrix * RotationAxis;

			Vector3d torque = hingeAxis * torqueAxis1 + rotationAxis * torqueAxis2;

            ShapeA.SetTorque(ShapeA.TorqueValue + torque);
            ShapeB.SetTorque(ShapeB.TorqueValue - torque);
		}
        
        #region NotImplementedMethods

        public override void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

		#region Private Static Methods

		List<JacobianConstraint> GetAngularLimit(
			IShape simulationObjectA,
			IShape simulationObjectB,
			Vector3d hingeAxis,
			Vector3d rotationAxis,
            double errorReduction)
		{
			var angularConstraint = new List<JacobianConstraint>();

			if (AngularLimitMin1.HasValue &&
				AngularLimitMax1.HasValue)
			{
				double angle1 = GetAngle1(
					hingeAxis,
					rotationAxis,
					HingeAxis,
					simulationObjectA.RotationStatus,
					RelativeOrientation1);

				JacobianConstraint? jContact = 
					JacobianCommon.GetAngularLimit (
						angle1,
                        errorReduction,
						0.0,
						simulationObjectA,
						simulationObjectB,
						hingeAxis,
						AngularLimitMin1.Value,
						AngularLimitMax1.Value);
				
				if (jContact != null)
					angularConstraint.Add (jContact.Value);
			}

			if (AngularLimitMin2.HasValue &&
				AngularLimitMax2.HasValue)
			{

				double angle2 = GetAngle2(
					hingeAxis,
					rotationAxis,
					RotationAxis,
					simulationObjectB.RotationStatus,
					RelativeOrientation2);

				JacobianConstraint? jContact = 
					JacobianCommon.GetAngularLimit (
						angle2,
                        errorReduction,
						0.0,
						simulationObjectA,
						simulationObjectB,
						rotationAxis,
						AngularLimitMin2.Value,
						AngularLimitMax2.Value);
				
				if (jContact != null)
					angularConstraint.Add (jContact.Value);
			}

			return angularConstraint;
		}

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

