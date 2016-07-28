using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;


namespace MonoPhysicsEngine
{
	public class SliderConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		const JointType jointType = JointType.Slider;

		int IndexA;
		int IndexB;
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
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 sliderAxis,
			double restoreCoefficient,
			double springCoefficient)
		{
			IndexA = indexA;
			IndexB = indexB;
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;
			SliderAxis = -1.0 * sliderAxis.Normalize ();

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			Vector3 relativePos = objectA.RotationMatrix *
				(startAnchorPosition - objectA.StartPosition);

			AnchorPoint = relativePos + objectA.Position;

			StartErrorAxis1 = objectA.RotationMatrix.Transpose() *
									 (AnchorPoint - objectA.Position);

			StartErrorAxis2 = objectB.RotationMatrix.Transpose() *
									 (AnchorPoint - objectB.Position);

			RelativeOrientation = objectB.RotationStatus.Inverse() *
										 objectA.RotationStatus;
		}

		#endregion

		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the slider joint.
		/// </summary>
		/// <returns>The slider joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			var sliderConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [IndexA];
			SimulationObject simulationObjectB = simulationObjs [IndexB];

			AnchorPoint = (simulationObjectA.RotationMatrix *
						  (StartAnchorPoint - simulationObjectA.StartPosition)) +
						  simulationObjectA.Position;	

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

			double constraintLimit = RestoreCoefficient * 2.0 * angularError.x;

			//DOF 1

			sliderConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = RestoreCoefficient * 2.0 * angularError.y;

			sliderConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = RestoreCoefficient * 2.0 * angularError.z;

			sliderConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (0.0, 0.0, 1.0),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = RestoreCoefficient * Vector3.Dot (t1,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
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
				ConstraintType.Joint));

			//DOF 5

			constraintLimit = RestoreCoefficient * Vector3.Dot (t2,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
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
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			// Limit extraction
			if (LinearLimitMin.HasValue &&
				LinearLimitMax.HasValue)
			{

				sliderConstraints.Add (
					JacobianCommon.GetLinearLimit(
						IndexA,
						IndexB,
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						r1,
						r2,
						RestoreCoefficient,
						SpringCoefficient,
						LinearLimitMin.Value,
						LinearLimitMax.Value));
			}

			#endregion

			#region Motor Constraint

			if (ForceLimit.HasValue &&
				SpeedValue.HasValue)
			{
				sliderConstraints.Add (JacobianCommon.GetDOF (
					IndexA,
					IndexB,
					sliderAxis,
					-1.0 * sliderAxis,
					new Vector3(),
					new Vector3(),
					simulationObjectA,
					simulationObjectB,
					SpeedValue.Value,
					0.0,
					SpringCoefficient,
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
			return IndexA;
		}

		public int GetObjectIndexB()
		{
			return IndexB;
		}

		public void SetObjectIndexA(int index)
		{
			IndexA = index;
		}

		public void SetObjectIndexB(int index)
		{
			IndexB = index;
		}

		public JointType GetJointType()
		{
			return jointType;
		}

		public Vector3 GetAnchorPosition()
		{
			return AnchorPoint;
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

		void IConstraint.AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

	}
}

