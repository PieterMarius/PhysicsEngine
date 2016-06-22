using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class PistonConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		const JointType jointType = JointType.Piston;

		readonly int IndexA;
		readonly int IndexB;
		readonly double SpringCoefficient;
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 PistonAxis;

		double? AngularLimitMin = null;
		double? AngularLimitMax = null;

		double? LinearLimitMin = null;
		double? LinearLimitMax = null;

		double? LinearSpeedValue = null;
		double? LinearForceLimit = null;
		double? AngularSpeedValue = null;
		double? AngularForceLimit = null;

		double RestoreCoefficient;
		Vector3 AnchorPoint;
		Vector3 StartErrorAxis1;
		Vector3 StartErrorAxis2;
		Quaternion RelativeOrientation;

		#endregion

		#region Constructor

		public PistonConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 pistonAxis,
			double restoreCoefficient,
			double springCoefficient)
		{
			IndexA = indexA;
			IndexB = indexB;
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;

			PistonAxis = -1.0 * pistonAxis.Normalize ();

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
		/// Builds the piston joint.
		/// </summary>
		/// <returns>The piston joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			var pistonConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [IndexA];
			SimulationObject simulationObjectB = simulationObjs [IndexB];

			this.AnchorPoint = (simulationObjectA.RotationMatrix *
								(StartAnchorPoint -
								simulationObjectA.StartPosition)) +
								simulationObjectA.Position;

			#region Init Linear

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * PistonAxis;

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

			Vector3 angularError = sliderAxis.Cross (
				                       (simulationObjectB.RotationMatrix * PistonAxis));

			#region Jacobian Constraint

			#region Base Constraints

			//DOF 1

			double angularLimit = RestoreCoefficient *
				t1.Dot (angularError);

			pistonConstraints.Add (
				JacobianCommon.GetDOF (
					IndexA, 
					IndexB, 
					new Vector3(), 
					new Vector3(), 
					1.0 * t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit,
					SpringCoefficient,
					0.0,
					ConstraintType.Joint));

			//DOF 2

			angularLimit = RestoreCoefficient *
				t2.Dot (angularError);

			pistonConstraints.Add (
				JacobianCommon.GetDOF (
					IndexA, 
					IndexB, 
					new Vector3(), 
					new Vector3(), 
					1.0 * t2, 
					-1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit,
					SpringCoefficient,
					0.0,
					ConstraintType.Joint));

			//DOF 3

			double constraintLimit = RestoreCoefficient * Vector3.Dot (t1,linearError);

			pistonConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = this.RestoreCoefficient * Vector3.Dot (t2,linearError);

			pistonConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				t2,
				-1.0 * t2,
				Vector3.Cross (r1, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				SpringCoefficient,
				0.0,
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			pistonConstraints.AddRange(getLinearLimit(
				simulationObjectA,
				simulationObjectB,
				sliderAxis,
				r1,
				r2));

			pistonConstraints.AddRange(getAnguarLimit(
				simulationObjectA,
				simulationObjectB,
				sliderAxis));

			#endregion

			#region Motor Constraint

			pistonConstraints.AddRange(getMotorConstraint(
				simulationObjectA,
				simulationObjectB,
				sliderAxis));

			#endregion

			#endregion

			return pistonConstraints;
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

		public JointType GetJointType()
		{
			return jointType;
		}

		public Vector3 GetAnchorPosition()
		{
			return this.AnchorPoint;
		}

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			AngularLimitMin = angularLimitMin;
			AngularLimitMax = angularLimitMax;
		}

		public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			LinearLimitMin = linearLimitMin;
			LinearLimitMax = linearLimitMax;
		}

		public void SetAxis1Motor(double speedValue, double forceLimit)
		{
			LinearSpeedValue = speedValue;
			LinearForceLimit = forceLimit;
		}

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

		public void AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			throw new NotImplementedException();
		}

		#region NotImplementedMethod

		void IConstraint.SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			throw new NotSupportedException();
		}

		void IConstraint.SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

		#region Private Methods

		List<JacobianContact> getLinearLimit(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Vector3 sliderAxis,
			Vector3 r1,
			Vector3 r2)
		{
			var linearConstraints = new List<JacobianContact>();

			if (LinearLimitMin.HasValue &&
				LinearLimitMax.HasValue)
			{
				linearConstraints.Add(
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

			return linearConstraints;
		}

		List<JacobianContact> getAnguarLimit(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Vector3 sliderAxis)
		{
			var angularConstraints = new List<JacobianContact>();

			if (AngularLimitMin.HasValue &&
				AngularLimitMax.HasValue)
			{
				double angle = JacobianCommon.GetAngle(
					simulationObjectA,
					simulationObjectB,
					this.RelativeOrientation,
					this.PistonAxis);

				angularConstraints.Add(
					JacobianCommon.GetAngularLimit(
						IndexA,
						IndexB,
						angle,
						RestoreCoefficient,
						SpringCoefficient,
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						AngularLimitMin.Value,
						AngularLimitMax.Value));
			}

			return angularConstraints;
		}

		List<JacobianContact> getMotorConstraint(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Vector3 sliderAxis)
		{
			var motorConstraints = new List<JacobianContact>();

			if (LinearForceLimit.HasValue &&
				LinearSpeedValue.HasValue)
			{
				motorConstraints.Add(JacobianCommon.GetDOF(
					IndexA,
					IndexB,
					sliderAxis,
					-1.0 * sliderAxis,
					new Vector3(),
					new Vector3(),
					simulationObjectA,
					simulationObjectB,
					LinearSpeedValue.Value,
					SpringCoefficient,
					LinearForceLimit.Value,
					ConstraintType.JointMotor));
			}

			if (AngularForceLimit.HasValue &&
			   AngularSpeedValue.HasValue)
			{
				motorConstraints.Add(JacobianCommon.GetDOF(
					IndexA,
					IndexB,
					new Vector3(),
					new Vector3(),
					sliderAxis,
					-1.0 * sliderAxis,
					simulationObjectA,
					simulationObjectB,
					AngularSpeedValue.Value,
					SpringCoefficient,
					AngularForceLimit.Value,
					ConstraintType.JointMotor));
			}

			return motorConstraints;
		}



		#endregion
	}
}

