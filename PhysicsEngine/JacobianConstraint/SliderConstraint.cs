using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;


namespace MonoPhysicsEngine
{
	public class SliderConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		private const JointType jointType = JointType.Slider;

		private readonly int IndexA;
		private readonly int IndexB;
		private readonly double C;
		private readonly double K;
		private readonly Vector3 StartAnchorPoint;
		private readonly Vector3 SliderAxis;
		private readonly Vector3 StartErrorAxis1;
		private readonly Vector3 StartErrorAxis2;
		private readonly Quaternion RelativeOrientation;

		private double? LinearLimitMin = null;
		private double? LinearLimitMax = null;
		private double? SpeedValue = null;
		private double? ForceLimit = null;
		private Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public SliderConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 sliderAxis,
			double K,
			double C)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;
			this.SliderAxis = -1.0 * sliderAxis.Normalize ();

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

			Vector3 relativePos = objectA.RotationMatrix *
				(startAnchorPosition - objectA.StartPosition);

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose() *
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
		/// Builds the slider joint.
		/// </summary>
		/// <returns>The slider joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			List<JacobianContact> sliderConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [IndexA];
			SimulationObject simulationObjectB = simulationObjs [IndexB];

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
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * 2.0 * angularError.y;

			sliderConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * 2.0 * angularError.z;

			sliderConstraints.Add (JacobianCommon.GetDOF (
				IndexA,
				IndexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (0.0, 0.0, 1.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = this.K * Vector3.Dot (t1,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
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

			//DOF 5

			constraintLimit = this.K * Vector3.Dot (t2,linearError);

			sliderConstraints.Add (JacobianCommon.GetDOF (
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

			// Limit extraction
			if (this.LinearLimitMin.HasValue &&
				this.LinearLimitMax.HasValue)
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
						this.K,
						C,
						this.LinearLimitMin.Value,
						this.LinearLimitMax.Value));
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
					this.SpeedValue.Value,
					C,
					this.ForceLimit.Value,
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

		public JointType GetJointType()
		{
			return jointType;
		}

		public Vector3 GetAnchorPosition()
		{
			return this.AnchorPoint;
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

		public void AddTorque(SimulationObject[] objects, double torqueAxis1, double torqueAxis2)
		{
			throw new NotImplementedException();
		}

		#endregion

		#endregion

		#endregion

	}
}

