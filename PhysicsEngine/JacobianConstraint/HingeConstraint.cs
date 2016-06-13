using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class HingeConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		public readonly double C;
		public readonly double K;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 StartErrorAxis1;
		public readonly Vector3 StartErrorAxis2;
		public readonly Quaternion RelativeOrientation;
		public readonly Vector3 HingeAxis;
		public readonly double? AngularLimitMin = null;
		public readonly double? AngularLimitMax = null;
		public readonly double? SpeedLimit = null;
		public readonly double? ForceLimit = null;

		private Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public HingeConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			double K,
			double C)
		{
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose () *
			                        (this.AnchorPoint - objectB.Position);

			this.RelativeOrientation = objectB.RotationStatus.Inverse () *
										objectA.RotationStatus;

			this.HingeAxis = hingeAxis.Normalize ();
		}

		public HingeConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			double K,
			double C,
			double? angularLimitMin,
			double? angularLimitMax,
			double? speedLimit = null,
			double? forceLimit = null)
			:this (objectA, objectB, startAnchorPosition, hingeAxis, K, C)
			
		{
			this.AngularLimitMin = angularLimitMin;
			this.AngularLimitMax = angularLimitMax;
			this.SpeedLimit = speedLimit;
			this.ForceLimit = forceLimit;
		}

		#endregion

		#region Public Methods

		/// <summary>
		/// Builds the hinge joint.
		/// </summary>
		/// <returns>The hinge joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(
			int indexA,
			int indexB,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> hingeConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			this.AnchorPoint = (simulationObjectA.RotationMatrix *
								(this.StartAnchorPoint -
								simulationObjectA.StartPosition)) +
								simulationObjectA.Position;

			#region Init Linear

			Vector3 axisRotated = simulationObjectA.RotationMatrix * this.HingeAxis;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (axisRotated).Normalize ();
			Vector3 t2 = Vector3.Cross (axisRotated, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				this.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				this.StartErrorAxis2;

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
				this.RelativeOrientation);

			#endregion

			#region Jacobian Constraint

			//DOF 1

			double constraintLimit = this.K * linearError.x;

			hingeConstraints.Add (JacobianCommon.GetDOF(
				indexA,
				indexB,
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (-skewP1.r1c1, -skewP1.r1c2, -skewP1.r1c3),
				new Vector3 (skewP2.r1c1,skewP2.r1c2,skewP2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * linearError.y;

			hingeConstraints.Add (JacobianCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (-skewP1.r2c1, -skewP1.r2c2, -skewP1.r2c3),
				new Vector3 (skewP2.r2c1,skewP2.r2c2,skewP2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * linearError.z;

			hingeConstraints.Add (JacobianCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (-skewP1.r3c1, -skewP1.r3c2, -skewP1.r3c3),
				new Vector3 (skewP2.r3c1,skewP2.r3c2,skewP2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				C,
				0.0,
				ConstraintType.Joint));

			//DOF 4

			double angularLimit = this.K *
				t1.Dot (angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t1, 
					1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					C,
					0.0,
					ConstraintType.Joint));

			//DOF 5

			angularLimit = this.K *
				t2.Dot (angularError);

			hingeConstraints.Add (
				JacobianCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t2, 
					1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit,
					C,
					0.0,
					ConstraintType.Joint));

			#region Limit Constraints 

			if (this.AngularLimitMin.HasValue && 
				this.AngularLimitMax.HasValue)
			{
				double angle = JacobianCommon.GetAngle (
					simulationObjectA,
					simulationObjectB,
					this.RelativeOrientation,
					this.HingeAxis);

				hingeConstraints.Add(JacobianCommon.GetAngularLimit (
					indexA, 
					indexB, 
					angle,
					this.K,
					C,
					simulationObjectA, 
					simulationObjectB, 
					axisRotated,
					this.AngularLimitMin.Value,
					this.AngularLimitMax.Value));
			}

			#endregion

			#region Motor Contraint

			if(this.SpeedLimit.HasValue &&
				this.ForceLimit.HasValue)
			{
				hingeConstraints.Add (
					JacobianCommon.GetDOF (
						indexA, 
						indexB, 
						new Vector3(), 
						new Vector3(), 
						-1.0 * axisRotated, 
						1.0 * axisRotated, 
						simulationObjectA, 
						simulationObjectB, 
						this.SpeedLimit.Value,
						C,
						this.ForceLimit.Value,
						ConstraintType.JointMotor));
			}

			#endregion

			#endregion

			return hingeConstraints;
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

