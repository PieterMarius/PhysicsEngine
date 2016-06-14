using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class UniversalConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		public readonly double C;
		public readonly double K;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 HingeAxis;
		public readonly Vector3 RotationAxis;
		public readonly double? AngularLimitMin1 = null;
		public readonly double? AngularLimitMax1 = null;
		public readonly double? AngularLimitMin2 = null;
		public readonly double? AngularLimitMax2 = null;

		private Vector3 AnchorPoint;
		private readonly Vector3 StartErrorAxis1;
		private readonly Vector3 StartErrorAxis2;
		private readonly Quaternion RelativeOrientation1;
		private readonly Quaternion RelativeOrientation2;

		#endregion

		#region Constructor

		public UniversalConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double K,
			double C)
		{
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;
			this.HingeAxis = hingeAxis.Normalize ();
			this.RotationAxis = rotationAxis.Normalize ();

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose () *
				(this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose () *
				(this.AnchorPoint - objectB.Position);

			Vector3 rHingeAxis = objectA.RotationMatrix * this.HingeAxis;
			Vector3 rRotationAxis = objectB.RotationMatrix * this.RotationAxis;

			this.RelativeOrientation1 = calculateRelativeOrientation (
				rHingeAxis,
				rRotationAxis,
				objectA.RotationStatus);

			this.RelativeOrientation2 = calculateRelativeOrientation (
				rRotationAxis,
				rHingeAxis,
				objectB.RotationStatus);
		}

		public UniversalConstraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double K,
			double C,
			double? angularLimitMin1,
			double? angularLimitMax1,
			double? angularLimitMin2,
			double? angularLimitMax2)
			:this(objectA, objectB, startAnchorPosition, hingeAxis, rotationAxis, K, C)
		{
			this.AngularLimitMin1 = angularLimitMin1;
			this.AngularLimitMax1 = angularLimitMax1;

			this.AngularLimitMin2 = angularLimitMin2;
			this.AngularLimitMax2 = angularLimitMax2;
		}

		#endregion


		#region Public Methods

		/// <summary>
		/// Builds the Universal joint.
		/// </summary>
		/// <returns>The Universal joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(
			int indexA,
			int indexB,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> hinge2Constraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			this.AnchorPoint = (simulationObjectA.RotationMatrix *
								(this.StartAnchorPoint -
								simulationObjectA.StartPosition)) +
								simulationObjectA.Position;

			#region Init Linear

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

			Vector3 hingeAxis = simulationObjectA.RotationMatrix * this.HingeAxis;
			Vector3 rotationAxis = simulationObjectB.RotationMatrix * this.RotationAxis;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3 t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

			#endregion

			#region Jacobian Constraint

			//DOF 1

			double constraintLimit = this.K * linearError.x;

			hinge2Constraints.Add (JacobianCommon.GetDOF(
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

			hinge2Constraints.Add (JacobianCommon.GetDOF (
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

			hinge2Constraints.Add (JacobianCommon.GetDOF (
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

			double angularLimit = this.K * (-k);

			hinge2Constraints.Add (
				JacobianCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit,
					C,
					0.0,
					ConstraintType.Joint));

			#region Limit Constraints 

			if (this.AngularLimitMin1.HasValue && 
				this.AngularLimitMax1.HasValue)
			{
				double angle1 = getAngle1(
					hingeAxis,
					rotationAxis,
					this.HingeAxis,
					simulationObjectA.RotationStatus,
					this.RelativeOrientation1);

				hinge2Constraints.Add(JacobianCommon.GetAngularLimit (
					indexA, 
					indexB, 
					angle1,
					this.K,
					C,
					simulationObjectA, 
					simulationObjectB, 
					hingeAxis,
					this.AngularLimitMin1.Value,
					this.AngularLimitMax1.Value));
			}

			if (this.AngularLimitMin2.HasValue &&
				this.AngularLimitMax2.HasValue)
			{
				
				double angle2 = getAngle2 (
					hingeAxis,
					rotationAxis,
					this.RotationAxis,
					simulationObjectB.RotationStatus,
					this.RelativeOrientation2);

				hinge2Constraints.Add(JacobianCommon.GetAngularLimit (
					indexA, 
					indexB, 
					angle2,
					this.K,
					C,
					simulationObjectA, 
					simulationObjectB, 
					rotationAxis,
					this.AngularLimitMin2.Value,
					this.AngularLimitMax2.Value));

			}

			#endregion

			#endregion

			return hinge2Constraints;
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

		#region Private Static Methods

		//TODO verificare se è possibile fondere in un unico metodo
		private double getAngle1(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix (axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion (rotationMatrix);

			Quaternion mult1 = Quaternion.Multiply1 (rotationStatus, rotationQ);
			Quaternion mult2 = Quaternion.Multiply2 (mult1, startRelativeRotation);

			Vector3 quaternionVectorPart = new Vector3 (
				mult2.b,
				mult2.c,
				mult2.d);

			return JacobianCommon.GetRotationAngle (quaternionVectorPart, mult2.a, startAxis);
		}

		private double getAngle2(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix (axis2, axis1);
			Quaternion rotationQ = Quaternion.GetQuaternion (rotationMatrix);

			Quaternion mult1 = Quaternion.Multiply1 (rotationStatus, rotationQ);
			Quaternion mult2 = Quaternion.Multiply2 (mult1, startRelativeRotation);

			Vector3 quaternionVectorPart = new Vector3 (
				mult2.b,
				mult2.c,
				mult2.d);

			return - JacobianCommon.GetRotationAngle (quaternionVectorPart, mult2.a, startAxis);
		}

		private Quaternion calculateRelativeOrientation(
			Vector3 axis1,
			Vector3 axis2,
			Quaternion bodyRotationStatus)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix (axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion (rotationMatrix);

			return Quaternion.Multiply1 (bodyRotationStatus, rotationQ);
		}

		#endregion
	}
}

