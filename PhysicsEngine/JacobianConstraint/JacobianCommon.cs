using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public static class JacobianCommon
	{
		public static Vector3 GetFixedAngularError(
			SimulationObject objectA,
			SimulationObject objectB,
			Quaternion relativeOrientation)
		{
			Quaternion currentRelativeOrientation = objectB.RotationStatus.Inverse () *
			                                        objectA.RotationStatus;

			Quaternion relativeOrientationError = relativeOrientation.Inverse () *
			                                      currentRelativeOrientation;

			var angularError = new Vector3 (
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

		public static double GetAngle(
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB,
			Quaternion relativeRotation,
			Vector3 rotationAxis)
		{
			Quaternion currentRelativeOrientation = simulationObjectA.RotationStatus.Inverse () *
				simulationObjectB.RotationStatus;

			Quaternion relativeOrientation = relativeRotation.Inverse () *
				currentRelativeOrientation;

			var quaternionVectorPart = new Vector3 (
				relativeOrientation.b,
				relativeOrientation.c,
				relativeOrientation.d);

			quaternionVectorPart = simulationObjectA.RotationMatrix * quaternionVectorPart;

			return GetRotationAngle (
				quaternionVectorPart,
				relativeOrientation.a,
				rotationAxis);
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
			double constraintValue,
			double correctionValue,
			double cfm,
			double constraintLimit,
			ConstraintType type,
			int? contactReference = null,
			StartImpulseProperties startImpulseProperties = null)
		{
			double jacobianVelocityValue = linearComponentA.Dot (simulationObjectA.LinearVelocity) +
			                               linearComponentB.Dot (simulationObjectB.LinearVelocity) +
			                               angularComponentA.Dot (simulationObjectA.AngularVelocity) +
			                               angularComponentB.Dot (simulationObjectB.AngularVelocity);

			jacobianVelocityValue -= constraintValue;

			if (startImpulseProperties == null)
				startImpulseProperties = new StartImpulseProperties(0.0);

			return new JacobianContact (
				indexA,
				indexB,
				contactReference,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				type,
				jacobianVelocityValue,
				correctionValue,
				cfm,
				constraintLimit,
				startImpulseProperties);
		}

		public static JacobianContact GetLinearLimit (
			int indexA, 
			int indexB, 
			SimulationObject simulationObjectA, 
			SimulationObject simulationObjectB, 
			Vector3 sliderAxis,
			Vector3 r1, 
			Vector3 r2,
			double K,
			double C,
			double linearLimitMin,
			double linearLimitMax)
		{
			double sliderDistance = Math.Abs((simulationObjectB.Position - simulationObjectA.Position).Dot (sliderAxis));

			if (sliderDistance < linearLimitMin) 
			{
				double linearLimit = K *
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
					0.0,
					linearLimit,
					C,
					0.0,
					ConstraintType.JointLimit);
			}
			else if (sliderDistance > linearLimitMax) 
			{
				double linearLimit = K *
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
					0.0,
					linearLimit,
					C,
					0.0,
					ConstraintType.JointLimit);
			}

			return new JacobianContact ();
		}

		public static JacobianContact? GetAngularLimit (
			int indexA, 
			int indexB, 
			double angle,
			double K,
			double C,
			SimulationObject simulationObjectA, 
			SimulationObject simulationObjectB,
			Vector3 rotationAxis,
			double angularLimitMin,
			double angularLimitMax)
		{
			if (angle > angularLimitMax) {

				double angularLimit = 
					K *
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
					0.0,
					angularLimit, 
					C,
					0.0,
					ConstraintType.JointLimit);

			} 

			if (angle < angularLimitMin) 
			{

				double angularLimit = 
					K *
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
					0.0,
					angularLimit,
					C,
					0.0,
					ConstraintType.JointLimit);
			}

			return null;
		}


	}
}

