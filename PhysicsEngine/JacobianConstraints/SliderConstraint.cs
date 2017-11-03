using System;
using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;


namespace SharpPhysicsEngine
{
	public class SliderConstraint: IConstraint, IConstraintBuilder
	{
		#region Private Fields

		const JointType jointType = JointType.Slider;

        readonly Vector3 xVec = new Vector3(1.0, 0.0, 0.0);
        readonly Vector3 xVecNeg = new Vector3(-1.0, 0.0, 0.0);
        readonly Vector3 yVec = new Vector3(0.0, 1.0, 0.0);
        readonly Vector3 yVecNeg = new Vector3(0.0, -1.0, 0.0);
        readonly Vector3 zVec = new Vector3(0.0, 0.0, 1.0);
        readonly Vector3 zVecNeg = new Vector3(0.0, 0.0, -1.0);

        IShape ShapeA;
        IShape ShapeB;
        int KeyIndex;
		readonly double SpringCoefficient;
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 SliderAxis;
		readonly Vector3 StartErrorAxis1;
		readonly Vector3 StartErrorAxis2;
		readonly Quaternion RelativeOrientation;

		double? LinearLimitMin;
		double? LinearLimitMax;
		double? SpeedValue;
		double? ForceLimit;
		Vector3 AnchorPoint;
		double RestoreCoefficient;

		#endregion

		#region Constructor

		public SliderConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3 startAnchorPosition,
			Vector3 sliderAxis,
			double restoreCoefficient,
			double springCoefficient)
		{
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = this.GetHashCode();
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;
			SliderAxis = -1.0 * sliderAxis.Normalize ();

			Vector3 relativePos = ShapeA.RotationMatrix *
				(startAnchorPosition - ShapeA.StartPosition);

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
		public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
		{
			var sliderConstraints = new List<JacobianConstraint> ();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            			
			#region Init Linear

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * SliderAxis;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3 t2 = Vector3.Cross (sliderAxis, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

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

			#region Base Constraints

			ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			double constraintLimit = RestoreCoefficient * 2.0 * angularError.x;

			//DOF 1

			sliderConstraints.Add (JacobianCommon.GetDOF (
                xVec,
                xVecNeg,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = RestoreCoefficient * 2.0 * angularError.y;

			sliderConstraints.Add (JacobianCommon.GetDOF (
                yVec,
                yVecNeg,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = RestoreCoefficient * 2.0 * angularError.z;

			sliderConstraints.Add (JacobianCommon.GetDOF (
                zVec,
                zVecNeg,
                simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 4

			constraintLimit = RestoreCoefficient * Vector3.Dot (t1,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 5

			constraintLimit = RestoreCoefficient * Vector3.Dot (t2,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				t2,
				-1.0 * t2,
				Vector3.Cross (r1, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
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
						RestoreCoefficient,
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
					new Vector3(),
					new Vector3(),
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

		public void SetAxis1Motor(double speedValue, double forceLimit)
		{
			SpeedValue = speedValue;
			ForceLimit = forceLimit;
		}

		public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			LinearLimitMin = linearLimitMin;
			LinearLimitMax = linearLimitMax;
		}

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

		#region NotImplementedMethods

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

		void IConstraint.AddTorque(double torqueAxis1, double torqueAxis2)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

	}
}

