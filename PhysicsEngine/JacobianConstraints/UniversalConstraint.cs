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
    internal sealed class UniversalConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		const JointType jointType = JointType.Universal;

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
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 HingeAxis;
		readonly Vector3 RotationAxis;
		readonly Vector3 StartErrorAxis1;
		readonly Vector3 StartErrorAxis2;
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
		Vector3 AnchorPoint;
		double RestoreCoefficient;

		#endregion

		#region Constructor

		public UniversalConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double restoreCoefficient,
			double springCoefficient)
		{
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = this.GetHashCode();
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;
			HingeAxis = hingeAxis.Normalize ();
			RotationAxis = rotationAxis.Normalize ();

			Vector3 relativePos = startAnchorPosition - ShapeA.StartPosition;
			relativePos = ShapeA.RotationMatrix * relativePos;

			AnchorPoint = relativePos + ShapeA.Position;

			StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeA.Position);

			StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeB.Position);

			Vector3 rHingeAxis = ShapeA.RotationMatrix * HingeAxis;
			Vector3 rRotationAxis = ShapeB.RotationMatrix * RotationAxis;

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
		public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
		{
			var universalConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;

			AnchorPoint = (simulationObjectA.RotationMatrix *
						  (StartAnchorPoint - simulationObjectA.StartPosition)) +
						  simulationObjectA.Position;

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Matrix3x3 skewP1 = Matrix3x3.GetSkewSymmetricMatrix (r1);
			Matrix3x3 skewP2 = Matrix3x3.GetSkewSymmetricMatrix (r2);

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 hingeAxis = simulationObjectA.RotationMatrix * HingeAxis;
			Vector3 rotationAxis = simulationObjectB.RotationMatrix * RotationAxis;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3 t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

			#endregion

			#region Jacobian Constraint

			#region Base Constraint

			ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double constraintLimit = RestoreCoefficient * linearError.x;

			universalConstraints.Add (JacobianCommon.GetDOF(
                xVec,
                xVecNeg,
                new Vector3 (-skewP1.r1c1, -skewP1.r1c2, -skewP1.r1c3),
				new Vector3 (skewP2.r1c1,skewP2.r1c2,skewP2.r1c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = RestoreCoefficient * linearError.y;

			universalConstraints.Add (JacobianCommon.GetDOF (
                yVec,
                yVecNeg,
                new Vector3 (-skewP1.r2c1, -skewP1.r2c2, -skewP1.r2c3),
				new Vector3 (skewP2.r2c1,skewP2.r2c2,skewP2.r2c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = RestoreCoefficient * linearError.z;

			universalConstraints.Add (JacobianCommon.GetDOF (
                zVec,
                zVecNeg,
                new Vector3 (-skewP1.r3c1, -skewP1.r3c2, -skewP1.r3c3),
				new Vector3 (skewP2.r3c1,skewP2.r3c2,skewP2.r3c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 4

			double angularLimit = RestoreCoefficient * (-k);

			universalConstraints.Add (
				JacobianCommon.GetDOF (
					t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit,
					SpringCoefficient,
					0.0,
					constraintType));

			#endregion

			#region Limit Constraints 

			universalConstraints.AddRange(GetAngularLimit(
				simulationObjectA,
				simulationObjectB,
				hingeAxis,
				rotationAxis));

			#endregion

			#endregion

			return universalConstraints;
		}

		#endregion

		#region IConstraint

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

		public JointType GetJointType()
		{
			return jointType;
		}

		public Vector3 GetAnchorPosition()
		{
			return (ShapeA.RotationMatrix *
                   (StartAnchorPoint - ShapeA.StartPosition)) +
                   ShapeA.Position; 
		}

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin1 = angularLimitMin;
			AngularLimitMax1 = angularLimitMax;
		}

		public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin2 = angularLimitMin;
			AngularLimitMax2 = angularLimitMax;
		}

		public void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedHingeAxisLimit = speedValue;
			ForceHingeAxisLimit = forceLimit;
		}

		public void SetAxis2Motor(double speedValue, double forceLimit)
		{
			SpeedRotationAxisLimit = speedValue;
			ForceRotationAxisLimit = forceLimit;
		}

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

		public void AddTorque( 
			double torqueAxis1, 
			double torqueAxis2)
		{
			Vector3 hingeAxis = ShapeA.RotationMatrix * HingeAxis;
			Vector3 rotationAxis = ShapeB.RotationMatrix * RotationAxis;

			Vector3 torque = hingeAxis * torqueAxis1 + rotationAxis * torqueAxis2;

            ShapeA.SetTorque(ShapeA.TorqueValue + torque);
            ShapeB.SetTorque(ShapeB.TorqueValue - torque);
		}

        public void SetSpringCoefficient(double springCoefficient)
        {
            SpringCoefficient = springCoefficient;
        }

        #region NotImplementedMethods

        void IConstraint.SetLinearLimit(double linearLimitMin, double linearLimitMax)
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
			Vector3 hingeAxis,
			Vector3 rotationAxis)
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
						RestoreCoefficient,
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
						RestoreCoefficient,
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

