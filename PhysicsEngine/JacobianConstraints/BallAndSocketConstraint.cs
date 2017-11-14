using System;
using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine
{
    internal sealed class BallAndSocketConstraint : IConstraint, IConstraintBuilder
	{
		#region Fields

		const JointType jointType = JointType.BallAndSocket;

        readonly Vector3 xVec = new Vector3(1.0, 0.0, 0.0);
        readonly Vector3 xVecNeg = new Vector3(-1.0, 0.0, 0.0);
        readonly Vector3 yVec = new Vector3(0.0, 1.0, 0.0);
        readonly Vector3 yVecNeg = new Vector3(0.0, -1.0, 0.0);
        readonly Vector3 zVec = new Vector3(0.0, 0.0, 1.0);
        readonly Vector3 zVecNeg = new Vector3(0.0, 0.0, -1.0);

        IShape ShapeA;
        IShape ShapeB;
        int KeyIndex;
		double SpringCoefficient;
		readonly Vector3 StartAnchorPoint;
		readonly Vector3 StartErrorAxis1;
		readonly Vector3 StartErrorAxis2;

		double RestoreCoefficient;
		Vector3 AnchorPoint;

		#endregion

		#region Constructor

		public BallAndSocketConstraint(
            IShape shapeA,
            IShape shapeB,
            Vector3 startAnchorPosition,
			double restoreCoefficient,
			double springCoefficient)
		{
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = GetHashCode();
			RestoreCoefficient = restoreCoefficient;
			SpringCoefficient = springCoefficient;
			StartAnchorPoint = startAnchorPosition;

			Vector3 relativePos = startAnchorPosition - ShapeA.StartPosition;
			relativePos = ShapeA.RotationMatrix * relativePos;

			AnchorPoint = relativePos + ShapeA.Position;

			StartErrorAxis1 = ShapeA.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeA.Position);

			StartErrorAxis2 = ShapeB.RotationMatrix.Transpose() *
									 (AnchorPoint - ShapeB.Position);
		}

		#endregion

		#region Public Methods

		#region IConstraintBuider

		/// <summary>
		/// Builds the ball socket joint.
		/// </summary>
		/// <returns>The ball socket joint.</returns>
		/// <param name="simulationObjs">Simulation objects.</param>
		public List<JacobianConstraint> BuildJacobian(double? baumStabilization = null)
		{
			var ballSocketConstraints = new List<JacobianConstraint>();

			IShape simulationObjectA = ShapeA;
			IShape simulationObjectB = ShapeB;
            
            #region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
										  StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
										  StartErrorAxis2;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix();

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Jacobian Constraint

			double restoreCoeff = (baumStabilization.HasValue) ? baumStabilization.Value : RestoreCoefficient;

			double constraintLimit = restoreCoeff * linearError.x;

            ConstraintType constraintType = ConstraintType.Joint;

			if (SpringCoefficient > 0)
				constraintType = ConstraintType.SoftJoint;

			//DOF 1

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
                xVec,
                xVecNeg,
                new Vector3(-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3(skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 2

			constraintLimit = restoreCoeff * linearError.y;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
                yVec,
                yVecNeg,
                new Vector3(-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3(skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			//DOF 3

			constraintLimit = restoreCoeff * linearError.z;

			ballSocketConstraints.Add(JacobianCommon.GetDOF(
                zVec,
                zVecNeg,
                new Vector3(-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3(skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				0.0,
				constraintLimit,
				SpringCoefficient,
				0.0,
				constraintType));

			#endregion

			return ballSocketConstraints;
		}

		#endregion

		#region IConstraint

		public JointType GetJointType()
		{
			return jointType;
		}

		public int GetObjectIndexA()
		{
			return ShapeA.ID;
		}

		public int GetObjectIndexB()
		{
			return ShapeB.ID;
		}

		public int GetKeyIndex()
		{
			return KeyIndex;
		}

		public Vector3 GetAnchorPosition()
		{
			return (ShapeA.RotationMatrix *
                   (StartAnchorPoint - ShapeA.StartPosition)) +
                   ShapeA.Position; 
		}

		public void SetRestoreCoefficient(double restoreCoefficient)
		{
			RestoreCoefficient = restoreCoefficient;
		}

        public void SetSpringCoefficient(double springCoefficient)
        {
            SpringCoefficient = springCoefficient;
        }

        #region NotSupportedMethods

        void IConstraint.SetAxis1Motor(double speedValue, double forceLimit)
		{
			throw new NotSupportedException();
		}

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

		void IConstraint.SetLinearLimit(double linearLimitMin, double linearLimitMax)
		{
			throw new NotSupportedException();
		}

		void IConstraint.AddTorque(double torqueAxis1, double torqueAxis2)
		{
			throw new NotSupportedException();
		}

		#endregion

		#endregion

		#endregion
	}
}

