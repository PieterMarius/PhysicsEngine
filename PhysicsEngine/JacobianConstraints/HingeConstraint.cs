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
    internal sealed class HingeConstraint: IConstraint, IConstraintBuilder
	{
		#region Private Fields

		const JointType jointType = JointType.Hinge;

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
		readonly Vector3 StartErrorAxis1;
		readonly Vector3 StartErrorAxis2;
		readonly Quaternion RelativeOrientation;

		double? AngularLimitMin;
		double? AngularLimitMax;
		double? SpeedValue;
		double? ForceLimit;

		double RestoreCoefficient;
		Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public HingeConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			double restoreCoefficient,
			double springCoefficient)
		{
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = GetHashCode();
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;

			Vector3 relativePos = startAnchorPosition - ShapeA.StartPosition;
			relativePos = ShapeA.RotationMatrix * relativePos;

			AnchorPoint = relativePos + ShapeA.Position;

			StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeA.Position);

			StartErrorAxis2 = shapeB.RotationMatrix.Transpose() *
									 (AnchorPoint - shapeB.Position);

			RelativeOrientation = shapeB.RotationStatus.Inverse () *
                                        ShapeA.RotationStatus;

			HingeAxis = hingeAxis.Normalize ();
		}

		#endregion

		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the hinge joint.
		/// </summary>
		/// <returns>The hinge joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
		{
			var hingeConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            
            #region Init Linear

			Vector3 axisRotated = simulationObjectA.RotationMatrix * HingeAxis;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (axisRotated).Normalize ();
			Vector3 t2 = Vector3.Cross (axisRotated, t1).Normalize ();

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

			Vector3 angularError = JacobianCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				RelativeOrientation);

			#endregion

			#region Jacobian Constraint

			#region Base Constraint

			ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			double constraintLimit = RestoreCoefficient * linearError.x;

			hingeConstraints.Add (JacobianCommon.GetDOF(
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

			hingeConstraints.Add (JacobianCommon.GetDOF (
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

			hingeConstraints.Add (JacobianCommon.GetDOF (
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

			double angularLimit = RestoreCoefficient *
				t1.Dot(angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
                    	-1.0 * t1, 
					1.0 * t1, 
					simulationObjectA, 
					simulationObjectB,
					0.0,
					angularLimit, 
					SpringCoefficient,
					0.0,
					constraintType));

			//DOF 5

			angularLimit = RestoreCoefficient *
				t2.Dot(angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
                    	-1.0 * t2, 
					1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					0.0,
					angularLimit,
					SpringCoefficient,
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
						RestoreCoefficient,
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

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

		public void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedValue = speedValue;
			ForceLimit = forceLimit;
		}

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin = angularLimitMin;
			AngularLimitMax = angularLimitMax;
		}

		public void AddTorque(double torqueAxis1, double torqueAxis2)
		{
			Vector3 hingeAxis = ShapeA.RotationMatrix * HingeAxis;

			Vector3 torque = hingeAxis * torqueAxis1;

            ShapeA.SetTorque(ShapeA.TorqueValue + torque);
            ShapeB.SetTorque(ShapeB.TorqueValue - torque);
		}

        public void SetSpringCoefficient(double springCoefficient)
        {
            SpringCoefficient = springCoefficient;
        }

        #region NotImplementedMethods

        void IConstraint.SetAxis2Motor(double speedValue, double forceLimit)
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

