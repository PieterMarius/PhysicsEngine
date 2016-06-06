using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;


namespace MonoPhysicsEngine
{
	public class SliderConstraint: IConstraint
	{
		#region Public Fields

		public readonly double C;
		public readonly double K;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 StartErrorAxis1;
		public readonly Vector3 StartErrorAxis2;
		public readonly Quaternion RelativeOrientation;
		public readonly Vector3 SliderAxis;
		public readonly double? LinearLimitMin = null;
		public readonly double? LinearLimitMax = null;
		public readonly double? SpeedLimit = null;
		public readonly double? ForceLimit = null;

		private Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public SliderConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 sliderAxis,
			double K,
			double C)
		{
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;
			this.SliderAxis = -1.0 * sliderAxis.Normalize ();

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

		public SliderConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 sliderAxis,
			double K,
			double C,
			double linearLimitMin,
			double linearLimitMax)
			:this(objectA, objectB, startAnchorPosition, sliderAxis, K, C)
		{
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;
		}

		public SliderConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 sliderAxis,
			double K,
			double C,
			double linearLimitMin,
			double linearLimitMax,
			double speedLimit,
			double forceLimit)
			:this(objectA, objectB, startAnchorPosition, sliderAxis, K, C)
		{
			this.LinearLimitMin = linearLimitMin;
			this.LinearLimitMax = linearLimitMax;
			this.SpeedLimit = speedLimit;
			this.ForceLimit = forceLimit;
		}

		#endregion

		#region Public Methods

		/// <summary>
		/// Builds the slider joint.
		/// </summary>
		/// <returns>The slider joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(
			int indexA,
			int indexB,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> sliderConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			this.AnchorPoint = (simulationObjectA.RotationMatrix *
								(this.StartAnchorPoint -
								simulationObjectA.StartPosition)) +
								simulationObjectA.Position;	

			#region Init Linear

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * this.SliderAxis;

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

			#region Init Angular

			Vector3 angularError = JacobianCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				this.RelativeOrientation);

			#endregion

			#region Jacobian Constraint

			#region Constraints

			double constraintLimit = this.K * 2.0 * angularError.x;

			//DOF 1

			sliderConstraints.Add (JacobianCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * 2.0 * angularError.y;

			sliderConstraints.Add (JacobianCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * 2.0 * angularError.z;

			sliderConstraints.Add (JacobianCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (0.0, 0.0, 1.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = this.K * Vector3.Dot (t1,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				indexA,
				indexB,
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				0.0,
				ConstraintType.Joint));

			//DOF 5

			constraintLimit = this.K * Vector3.Dot (t2,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
				indexA,
				indexB,
				t2,
				-1.0 * t2,
				Vector3.Cross (r1, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				0.0,
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			// Limit extraction
			if (this.LinearLimitMin.HasValue &&
				this.LinearLimitMax.HasValue)
			{

				sliderConstraints.Add (
					JacobianCommon.GetLinearLimit(
						indexA,
						indexB,
						simulationObjectA,
						simulationObjectB,
						sliderAxis,
						r1,
						r2,
						this.K,
						this.LinearLimitMin.Value,
						this.LinearLimitMax.Value));
			}

			#endregion

			#region Motor Constraint

			if (ForceLimit.HasValue &&
				SpeedLimit.HasValue)
			{
				sliderConstraints.Add (JacobianCommon.GetDOF (
					indexA,
					indexB,
					sliderAxis,
					-1.0 * sliderAxis,
					new Vector3(),
					new Vector3(),
					simulationObjectA,
					simulationObjectB,
					this.SpeedLimit.Value,
					this.ForceLimit.Value,
					ConstraintType.JointMotor));
			}

			#endregion

			#endregion

			return sliderConstraints;
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

