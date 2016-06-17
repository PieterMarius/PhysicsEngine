using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class PistonConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		private const JointType jointType = JointType.Piston;

		private readonly int IndexA;
		private readonly int IndexB;
		private readonly double C;
		private readonly double K;
		private readonly Vector3 StartAnchorPoint;
		private readonly Vector3 PistonAxis;

		private double? AngularLimitMin = null;
		private double? AngularLimitMax = null;
		private double? LinearLimitMin = null;
		private double? LinearLimitMax = null;

		private Vector3 AnchorPoint;
		private Vector3 StartErrorAxis1;
		private Vector3 StartErrorAxis2;
		private Quaternion RelativeOrientation;

		#endregion

		#region Constructor

		public PistonConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 pistonAxis,
			double K,
			double C)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;

			this.PistonAxis = -1.0 * pistonAxis.Normalize ();

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			Vector3 relativePos = objectA.RotationMatrix *
				(startAnchorPosition - objectA.StartPosition);

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectB.Position);

			this.RelativeOrientation = objectB.RotationStatus.Inverse () *
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
								(this.StartAnchorPoint -
								simulationObjectA.StartPosition)) +
								simulationObjectA.Position;

			#region Init Linear

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * this.PistonAxis;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3 t2 = Vector3.Cross (sliderAxis, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				this.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				this.StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			Vector3 angularError = sliderAxis.Cross (
				                       (simulationObjectB.RotationMatrix * this.PistonAxis));

			#region Jacobian Constraint

			#region Base Constraints

			//DOF 1

			double angularLimit = this.K *
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
					C,
					0.0,
					ConstraintType.Joint));

			//DOF 2

			angularLimit = this.K *
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
					C,
					0.0,
					ConstraintType.Joint));

			//DOF 3

			double constraintLimit = this.K * Vector3.Dot (t1,linearError);

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
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = this.K * Vector3.Dot (t2,linearError);

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
				C,
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
			throw new NotSupportedException();
		}

		public void SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

		#region NotImplementedMethod

		void IConstraint.SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion

		#region Private Methods

		private List<JacobianContact> getLinearLimit(
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
						K,
						C,
						LinearLimitMin.Value,
						LinearLimitMax.Value));
			}

			return linearConstraints;
		}

		private List<JacobianContact> getAnguarLimit(
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
						K,
						C,
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						AngularLimitMin.Value,
						AngularLimitMax.Value));
			}

			return angularConstraints;
		}

		public void AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			throw new NotImplementedException();
		}

		#endregion
	}
}

