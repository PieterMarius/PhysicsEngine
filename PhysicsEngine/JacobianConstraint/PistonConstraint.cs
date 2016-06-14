using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class PistonConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		public readonly double C;
		public readonly double K;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 PistonAxis;
		public readonly double? AngularLimitMin = null;
		public readonly double? AngularLimitMax = null;
		public readonly double? LinearLimitMin = null;
		public readonly double? LinearLimitMax = null;

		private Vector3 AnchorPoint;
		private Vector3 StartErrorAxis1;
		private Vector3 StartErrorAxis2;
		private Quaternion RelativeOrientation;

		#endregion

		#region Constructor

		public PistonConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 pistonAxis,
			double K,
			double C)
		{
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;

			this.PistonAxis = -1.0 * pistonAxis.Normalize ();

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

		public PistonConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 pistonAxis,
			double K,
			double C,
			double linearLimitMin,
			double linearLimitMax)
			:this(objectA, objectB, startAnchorPosition, pistonAxis, K, C)
		{
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;
		}
		
		public PistonConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 pistonAxis,
			double K,
			double C,
			double linearLimitMin,
			double linearLimitMax,
			double angularLimitMin,
			double angularLimitMax)
			:this(objectA, objectB, startAnchorPosition, pistonAxis, K, C)
		{
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;

			this.AngularLimitMin = angularLimitMin;
			this.AngularLimitMax = angularLimitMax;
		}

		#endregion

		#region Public Methods

		#region IConstraint

		/// <summary>
		/// Builds the piston joint.
		/// </summary>
		/// <returns>The piston joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(
			int indexA,
			int indexB,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> pistonConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

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
					indexA, 
					indexB, 
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
					indexA, 
					indexB, 
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
				indexA,
				indexB,
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
				indexA,
				indexB,
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

			if (LinearLimitMin.HasValue &&
				LinearLimitMax.HasValue)
			{
				pistonConstraints.Add(
					JacobianCommon.GetLinearLimit(
						indexA,
						indexB,
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						r1,
						r2,
						this.K,
						C,
						LinearLimitMin.Value,
						LinearLimitMax.Value));
			}

			if (AngularLimitMin.HasValue &&
				AngularLimitMax.HasValue)
			{
				double angle = JacobianCommon.GetAngle(
					simulationObjectA,
					simulationObjectB,
					this.RelativeOrientation,
					this.PistonAxis);

				pistonConstraints.Add(
					JacobianCommon.GetAngularLimit(
						indexA,
						indexB,
						angle,
						this.K,
						C,
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						AngularLimitMin.Value,
						AngularLimitMax.Value));
			}

			#endregion

			#endregion

			return pistonConstraints;
		}

		public Vector3 GetStartAnchorPosition()
		{
			return this.StartAnchorPoint;
		}

		public Vector3 GetAnchorPosition()
		{
			return this.AnchorPoint;
		}

		public void SetAxis1Motor(double speedValue, double forceLimit)
		{
			throw new NotImplementedException();
		}

		public void SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotImplementedException();
		}

		#endregion

		#endregion
	}
}

