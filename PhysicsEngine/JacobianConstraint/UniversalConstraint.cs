using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class UniversalConstraint: IConstraint, IConstraintBuilder
	{
		#region Public Fields

		private const JointType jointType = JointType.Universal;

		private readonly int IndexA;
		private readonly int IndexB;
		private readonly double C;
		private readonly double K;
		private readonly Vector3 StartAnchorPoint;
		private readonly Vector3 HingeAxis;
		private readonly Vector3 RotationAxis;
		private readonly Vector3 StartErrorAxis1;
		private readonly Vector3 StartErrorAxis2;
		private readonly Quaternion RelativeOrientation1;
		private readonly Quaternion RelativeOrientation2;

		private double? AngularLimitMin1 = null;
		private double? AngularLimitMax1 = null;
		private double? AngularLimitMin2 = null;
		private double? AngularLimitMax2 = null;

		private Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public UniversalConstraint(
			int indexA,
			int indexB,
			SimulationObject[] simulationObject,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double K,
			double C)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;
			this.HingeAxis = hingeAxis.Normalize ();
			this.RotationAxis = rotationAxis.Normalize ();

			SimulationObject objectA = simulationObject[IndexA];
			SimulationObject objectB = simulationObject[IndexB];

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

		#endregion


		#region Public Methods

		#region IConstraintBuilder

		/// <summary>
		/// Builds the Universal joint.
		/// </summary>
		/// <returns>The Universal joint.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs)
		{
			List<JacobianContact> hinge2Constraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [IndexA];
			SimulationObject simulationObjectB = simulationObjs [IndexB];

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
				IndexA,
				IndexB,
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
				IndexA,
				IndexB,
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
				IndexA,
				IndexB,
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
					IndexA, 
					IndexB, 
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
					IndexA, 
					IndexB, 
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
					IndexA, 
					IndexB, 
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
			throw new NotSupportedException();
		}

		public void SetAxis2Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

		public void AddTorque(double torqueAxis1, double torqueAxis2)
		{
			throw new NotSupportedException();
		}

		public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			this.AngularLimitMin1 = angularLimitMin;
			this.AngularLimitMax1 = angularLimitMax;
		}

		public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
		{
			this.AngularLimitMin2 = angularLimitMin;
			this.AngularLimitMax2 = angularLimitMax;
		}

		public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotImplementedException();
		}

		#endregion

		#endregion

		#region Private Static Methods

		private double getAngle2(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			return -getAngle1(axis2, axis1, startAxis, rotationStatus, startRelativeRotation);
		}

		private double getAngle1(
			Vector3 axis1,
			Vector3 axis2,
			Vector3 startAxis,
			Quaternion rotationStatus,
			Quaternion startRelativeRotation)
		{
			Matrix3x3 rotationMatrix = Matrix3x3.GetRotationMatrix(axis1, axis2);
			Quaternion rotationQ = Quaternion.GetQuaternion(rotationMatrix);

			Quaternion mult1 = Quaternion.Multiply1(rotationStatus, rotationQ);
			Quaternion mult2 = Quaternion.Multiply2(mult1, startRelativeRotation);

			Vector3 quaternionVectorPart = new Vector3(
				mult2.b,
				mult2.c,
				mult2.d);

			return JacobianCommon.GetRotationAngle(quaternionVectorPart, mult2.a, startAxis);
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

