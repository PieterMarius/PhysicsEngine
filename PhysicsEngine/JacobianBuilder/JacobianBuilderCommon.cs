using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public static class JacobianBuilderCommon
	{
		public static Vector3 GetFixedAngularError(
			SimulationObject objectA,
			SimulationObject objectB,
			Joint simulationJoint)
		{
			Quaternion currentRelativeOrientation = objectB.RotationStatus.Inverse () *
			                                        objectA.RotationStatus;

			Quaternion relativeOrientationError = simulationJoint.RelativeRotation1.Inverse () *
			                                      currentRelativeOrientation;

			Vector3 angularError = new Vector3 (
				relativeOrientationError.b, 
				relativeOrientationError.c, 
				relativeOrientationError.d);

			if (relativeOrientationError.a < 0.0) 
			{
				angularError = new Vector3 (
					-angularError.x,
					-angularError.y,
					-angularError.z);
			}

			return objectA.RotationMatrix * angularError;
		}

		public static double GetRotationAngle(
			Vector3 rotationStatus,
			double rotationValue,
			Vector3 rotationAxis)
		{
			double angle = 0.0;

			if (rotationStatus.Dot (rotationAxis) >= 0.0) 
			{
				angle = 2.0 * Math.Atan2 (rotationStatus.Length (), rotationValue);
			} 
			else 
			{
				angle = 2.0 * Math.Atan2 (rotationStatus.Length (), -rotationValue);
			}

			return (angle > Math.PI) ? angle - 2.0 * Math.PI : angle;
		}

		public static JacobianContact GetDOF(
			int indexA,
			int indexB,
			Vector3 linearComponentA,
			Vector3 linearComponentB,
			Vector3 angularComponentA,
			Vector3 angularComponentB,
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			double? constraintLimitMin,
			double? constraintLimitMax,
			ConstraintType type,
			int? contactReference = null)
		{
			double jacobianVelocityValue = linearComponentA.Dot (simulationObjectA.LinearVelocity) +
				linearComponentB.Dot (simulationObjectB.LinearVelocity) +
				angularComponentA.Dot (simulationObjectA.AngularVelocity) +
				angularComponentB.Dot (simulationObjectB.AngularVelocity);

			double B = jacobianVelocityValue;

			if (!constraintLimitMin.HasValue &&
				!constraintLimitMax.HasValue) 
			{
				constraintLimitMin = double.MinValue;
				constraintLimitMax = double.MaxValue;

			} 
			else if (constraintLimitMin == constraintLimitMax) 
			{
				B = jacobianVelocityValue - constraintLimitMin.Value;
				constraintLimitMin = double.MinValue;
				constraintLimitMax = double.MaxValue;
			}

			return new JacobianContact (
				indexA,
				indexB,
				contactReference,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				type,
				B,
				constraintLimitMin.Value,
				constraintLimitMax.Value,
				0.0);
		}

		public static JacobianContact GetLinearLimit (
			int indexA, 
			int indexB, 
			Joint simulationJoint,
			SimulationObject simulationObjectA, 
			SimulationObject simulationObjectB, 
			Vector3 sliderAxis,
			Vector3 r1, 
			Vector3 r2,
			double linearLimitMin,
			double linearLimitMax)
		{

			double sliderDistance = Math.Abs((simulationObjectB.Position - simulationObjectA.Position).Dot (sliderAxis));

			//Console.WriteLine ("Slider distance: " + sliderDistance);

			if (sliderDistance < linearLimitMin) 
			{

				double linearLimit = simulationJoint.K *
					(linearLimitMin - sliderDistance);

				return GetDOF (
					indexA, 
					indexB, 
					sliderAxis, 
					-1.0 * sliderAxis, 
					-1.0 * Vector3.Cross (r1, sliderAxis), 
					Vector3.Cross (r2, sliderAxis), 
					simulationObjectA, 
					simulationObjectB, 
					linearLimit, 
					linearLimit, 
					ConstraintType.JointLimit);
			}
			else if (sliderDistance > linearLimitMax) 
			{
				double linearLimit = simulationJoint.K *
					(sliderDistance - linearLimitMax);

				return GetDOF (
					indexA, 
					indexB, 
					-1.0 * sliderAxis, 
					sliderAxis, 
					Vector3.Cross (r1, sliderAxis), 
					-1.0 * Vector3.Cross (r2, sliderAxis), 
					simulationObjectA, 
					simulationObjectB, 
					linearLimit, 
					linearLimit, 
					ConstraintType.JointLimit);
			}

			return new JacobianContact ();
		}

		public static double GetAngle1(
			Vector3 axis1,
			Vector3 axis2,
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

			//quaternionVectorPart = simulationObjectA.RotationMatrix * quaternionVectorPart;
			//TODO work in progress

			return GetRotationAngle (quaternionVectorPart, mult2.a, new Vector3 ());
		}

		public static JacobianContact GetAngularLimit (
			int indexA, 
			int indexB, 
			Joint simulationJoint,  
			SimulationObject simulationObjectA, 
			SimulationObject simulationObjectB,
			Vector3 rotationAxis,
			double angularLimitMin,
			double angularLimitMax)
		{
			//TODO spostare la logica all'interno dei metodi di joint
			Quaternion currentRelativeOrientation = simulationObjectA.RotationStatus.Inverse () *
			                                        simulationObjectB.RotationStatus;

			Quaternion relativeOrientation = simulationJoint.RelativeRotation1.Inverse () *
			                                 currentRelativeOrientation;

			Vector3 quaternionVectorPart = new Vector3 (
				                               relativeOrientation.b,
				                               relativeOrientation.c,
				                               relativeOrientation.d);

			quaternionVectorPart = simulationObjectA.RotationMatrix * quaternionVectorPart;

			double angle = 
				GetRotationAngle (
					quaternionVectorPart,
					relativeOrientation.a,
					rotationAxis);

			if (angle > angularLimitMax) {

				double angularLimit = 
					simulationJoint.K *
					(angle - angularLimitMax);

				return GetDOF (
					indexA, 
					indexB, 
					new Vector3 (), 
					new Vector3 (), 
					rotationAxis, 
					-1.0 * rotationAxis, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.JointLimit);

			} 

			if (angle < angularLimitMin) 
			{

				double angularLimit = 
					simulationJoint.K *
					(angularLimitMin - angle);

				return GetDOF (
					indexA, 
					indexB, 
					new Vector3 (), 
					new Vector3 (), 
					-1.0 * rotationAxis, 
					rotationAxis, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.JointLimit);
			}

			return new JacobianContact ();
		}


	}
}

