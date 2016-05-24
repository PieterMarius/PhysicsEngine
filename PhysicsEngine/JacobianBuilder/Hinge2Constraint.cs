using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public sealed class Hinge2Constraint: IConstraint
	{
		#region Public Fields

		public readonly double C;
		public readonly double K;
		public readonly Vector3 StartAnchorPoint;
		public readonly Vector3 StartErrorAxis1;
		public readonly Vector3 StartErrorAxis2;
		public readonly Quaternion RelativeOrientation1;
		public readonly Quaternion RelativeOrientation2;
		public readonly Vector3 JointActDirection1;
		public readonly Vector3 JointActDirection2;
		public readonly Vector3 AngularLimitMin;
		public readonly Vector3 AngularLimitMax;

		public Vector3 AnchorPoint { get; private set; }

		#endregion

		#region Constructor

		public Hinge2Constraint(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 startAnchorPosition,
			Vector3 hingeAxis,
			Vector3 rotationAxis,
			double K,
			double C,
			double angularLimitMin = 0.0,
			double angularLimitMax = 0.0)
		{
			this.K = K;
			this.C = C;
			this.StartAnchorPoint = startAnchorPosition;
			this.JointActDirection1 = hingeAxis.Normalize ();
			this.JointActDirection2 = rotationAxis.Normalize ();

			Vector3 relativePos = startAnchorPosition - objectA.StartPosition;
			relativePos = objectA.RotationMatrix * relativePos;

			this.AnchorPoint = relativePos + objectA.Position;

			this.StartErrorAxis1 = objectA.RotationMatrix.Transpose () *
				(this.AnchorPoint - objectA.Position);

			this.StartErrorAxis2 = objectB.RotationMatrix.Transpose () *
				(this.AnchorPoint - objectB.Position);

			Vector3 rHingeAxis = objectA.RotationMatrix * this.JointActDirection1;
			Vector3 rRotationAxis = objectB.RotationMatrix * this.JointActDirection2;

			this.RelativeOrientation1 = calculateRelativeOrientation (
				rHingeAxis,
				rRotationAxis,
				objectA.RotationStatus);

			this.RelativeOrientation2 = calculateRelativeOrientation (
				rRotationAxis,
				rHingeAxis,
				objectB.RotationStatus); 

			if (angularLimitMin != angularLimitMax) {
				this.AngularLimitMin = hingeAxis * angularLimitMin;
				this.AngularLimitMin = hingeAxis * angularLimitMax;
			} 
			else 
			{
				this.AngularLimitMin = new Vector3 ();
				this.AngularLimitMax = new Vector3 ();
			}
		}

		#endregion


		#region Public Methods

		/// <summary>
		/// Builds the hinge2 joint.
		/// </summary>
		/// <returns>The hinge2 joint.</returns>
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

			Vector3 hingeAxis = simulationObjectA.RotationMatrix * this.JointActDirection1;

			Vector3 rotationAxis = simulationObjectB.RotationMatrix * this.JointActDirection2;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3 t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

			#endregion

			#region Jacobian Constraint

			//DOF 1

			double constraintLimit = this.K * linearError.x;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF(
				indexA,
				indexB,
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (-skewP1.r1c1, -skewP1.r1c2, -skewP1.r1c3),
				new Vector3 (skewP2.r1c1,skewP2.r1c2,skewP2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = this.K * linearError.y;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (-skewP1.r2c1, -skewP1.r2c2, -skewP1.r2c3),
				new Vector3 (skewP2.r2c1,skewP2.r2c2,skewP2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = this.K * linearError.z;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (-skewP1.r3c1, -skewP1.r3c2, -skewP1.r3c3),
				new Vector3 (skewP2.r3c1,skewP2.r3c2,skewP2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			double angularLimit = this.K * (-k);

			hingeConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			#region Limit Constraints 

			//Limit extraction
			double angularLimitMax = this.JointActDirection1.Dot (this.AngularLimitMax);
			double angularLimitMin = this.JointActDirection1.Dot (this.AngularLimitMin);

			double angle1 = getAngle1(
				hingeAxis,
				rotationAxis,
				this.JointActDirection1,
				simulationObjectA.RotationStatus,
				this.RelativeOrientation1);

			double angle2 = getAngle2 (
				hingeAxis,
				rotationAxis,
				this.JointActDirection2,
				simulationObjectB.RotationStatus,
				this.RelativeOrientation2);
				
			Console.WriteLine ("angle1 " + angle1);
			Console.WriteLine ("angle2 " + angle2);

			//			hingeConstraints.Add(JacobianBuilderCommon.GetAngularLimit (
			//				indexA, 
			//				indexB, 
			//				simulationJoint, 
			//				simulationObjectA, 
			//				simulationObjectB, 
			//				hingeAxis,
			//				angularLimitMin,
			//				angularLimitMax));

			#endregion

			#endregion

			return hingeConstraints;
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

			return JacobianBuilderCommon.GetRotationAngle (quaternionVectorPart, mult2.a, startAxis);
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

			return - JacobianBuilderCommon.GetRotationAngle (quaternionVectorPart, mult2.a, startAxis);
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

