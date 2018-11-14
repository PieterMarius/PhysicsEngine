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
    internal class SliderConstraint: Constraint
	{
		#region Private Fields

		const JointType jointType = JointType.Slider;
                
        readonly Vector3d StartAnchorPoint;
		readonly Vector3d SliderAxis;
		readonly Vector3d StartErrorAxis1;
		readonly Vector3d StartErrorAxis2;
		readonly Quaternion RelativeOrientation;

		double? LinearLimitMin;
		double? LinearLimitMax;
		double? SpeedValue;
		double? ForceLimit;
		Vector3d AnchorPoint;

        #endregion

        #region Constructor

        public SliderConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3d startAnchorPosition,
            Vector3d sliderAxis,
            double errorReductionParam,
            double springCoefficient)
            : base(shapeA, shapeB, errorReductionParam, springCoefficient)
        {
            StartAnchorPoint = startAnchorPosition;
            SliderAxis = -1.0 * sliderAxis.Normalize();

            Vector3d relativePos = ShapeA.RotationMatrix *
                (startAnchorPosition - ShapeA.InitCenterOfMass);

            AnchorPoint = relativePos + ShapeA.Position;

            StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeA.Position);

            StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
                                     (AnchorPoint - ShapeB.Position);

            RelativeOrientation = ShapeB.RotationStatus.Inverse() *
                                         ShapeA.RotationStatus;
        }

		#endregion

		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the slider joint.
		/// </summary>
		/// <returns>The slider joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public override List<JacobianConstraint> BuildJacobian(double timeStep, double? baumStabilization = null)
		{
			var sliderConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            			
			#region Init Linear

			Vector3d sliderAxis = GetSliderAxis();

			Vector3d t1 = GeometryUtils.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3d t2 = Vector3d.Cross (sliderAxis, t1).Normalize ();

			Vector3d r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3d r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

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

            #region Base Constraints

            ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			double constraintLimit = errorReduction * 2.0 * angularError.x;

			//DOF 1

			sliderConstraints.Add (JacobianCommon.GetDOF (
                xVecNeg,
                xVec,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = errorReduction * 2.0 * angularError.y;

			sliderConstraints.Add (JacobianCommon.GetDOF (
                yVecNeg,
                yVec,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = errorReduction * 2.0 * angularError.z;

			sliderConstraints.Add (JacobianCommon.GetDOF (
                zVecNeg,
                zVec,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 4

			constraintLimit = errorReduction * Vector3d.Dot (t1,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				t1,
				-1.0 * t1,
				Vector3d.Cross (r1, t1),
				-1.0 * Vector3d.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			//DOF 5

			constraintLimit = errorReduction * Vector3d.Dot (t2,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				t2,
				-1.0 * t2,
				Vector3d.Cross (r1, t2),
				-1.0 * Vector3d.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
                springCoefficient,
				0.0,
				constraintType));

			#endregion

			#region Limit Constraints 

			// Limit extraction
			if (LinearLimitMin.HasValue &&
				LinearLimitMax.HasValue)
			{

				sliderConstraints.Add (
					JacobianCommon.GetLinearLimit(
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						r1,
						r2,
                        errorReduction,
						0.0,
						LinearLimitMin.Value,
						LinearLimitMax.Value));
			}

			#endregion

			#region Motor Constraint

			if (ForceLimit.HasValue &&
				SpeedValue.HasValue)
			{
				sliderConstraints.Add (JacobianCommon.GetDOF (
					sliderAxis,
					-1.0 * sliderAxis,
					new Vector3d(),
					new Vector3d(),
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

			return sliderConstraints;
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

		public override void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			LinearLimitMin = linearLimitMin;
			LinearLimitMax = linearLimitMax;
		}

        public Vector3d GetSliderAxis()
        {
            return ShapeA.RotationMatrix * SliderAxis;
        }

        #region NotImplementedMethods

        public override void SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

        public override void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			throw new NotSupportedException();
		}

        public override void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
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

