using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class PistonConstraint: IConstraint
	{
		#region Public Fields

		public readonly double C;
		public readonly double K;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 StartErrorAxis1;
		public readonly Vector3 StartErrorAxis2;
		public readonly Quaternion RelativeOrientation;
		public readonly Vector3 PistonAxis;
		public readonly Vector3 AngularLimitMin = new Vector3 ();
		public readonly Vector3 AngularLimitMax = new Vector3 ();
		public readonly Vector3 LinearLimitMin = new Vector3 ();
		public readonly Vector3 LinearLimitMax = new Vector3();

		public Vector3 AnchorPoint { get; private set; }

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
			this.LinearLimitMin = this.PistonAxis * linearLimitMin;
			this.LinearLimitMax = this.PistonAxis * linearLimitMax;
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
			this.LinearLimitMin = this.PistonAxis * linearLimitMin;
			this.LinearLimitMax = this.PistonAxis * linearLimitMax;

			this.AngularLimitMin = this.PistonAxis * angularLimitMin;
			this.AngularLimitMax = this.PistonAxis * angularLimitMax;
		}

		#endregion

		#region Public Methods

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

			#region Constraints

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
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = this.K * Vector3.Dot (t2,linearError);

			Console.WriteLine ("linear error:" + constraintLimit);

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
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			// Limit extraction
			double linearLimitMin = this.PistonAxis.Dot (this.LinearLimitMin);
			double linearLimitMax = this.PistonAxis.Dot (this.LinearLimitMax);

			pistonConstraints.Add (
				JacobianCommon.GetLinearLimit(
					indexA,
					indexB,
					simulationObjectA,
					simulationObjectB,
					sliderAxis,
					r1,
					r2,
					this.K,
					linearLimitMin,
					linearLimitMax));

			// Limit extraction
			double angularLimitMin = this.PistonAxis.Dot (this.AngularLimitMin);
			double angularLimitMax = this.PistonAxis.Dot (this.AngularLimitMax);

			double angle = JacobianCommon.GetAngle (
				simulationObjectA,
				simulationObjectB,
				this.RelativeOrientation,
				this.PistonAxis);

			pistonConstraints.Add(
				JacobianCommon.GetAngularLimit (
					indexA, 
					indexB, 
					angle,
					this.K,
					simulationObjectA, 
					simulationObjectB, 
					sliderAxis,
					angularLimitMin,
					angularLimitMax));

			#endregion

			#endregion

			return pistonConstraints;
		}

		public void SetAnchorPosition(Vector3 position)
		{
			this.AnchorPoint = position;
		}

		public Vector3 GetStartAnchorPosition()
		{
			return this.StartAnchorPoint;
		}

		public Vector3 GetAnchorPosition()
		{
			return this.AnchorPoint;
		}

		#endregion
	}
}

