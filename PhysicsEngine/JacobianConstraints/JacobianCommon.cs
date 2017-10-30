using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
	public static class JacobianCommon
	{
		public static Vector3 GetFixedAngularError(
			IShape objectA,
			IShape objectB,
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

        public static Vector3 GetFixedAngularError(
            SoftShapePoint objectA,
            SoftShapePoint objectB,
            Quaternion relativeOrientation)
        {
            Quaternion currentRelativeOrientation = objectB.RotationStatus.Inverse() *
                                                    objectA.RotationStatus;

            Quaternion relativeOrientationError = relativeOrientation.Inverse() *
                                                  currentRelativeOrientation;

            var angularError = new Vector3(
                relativeOrientationError.b,
                relativeOrientationError.c,
                relativeOrientationError.d);

            if (relativeOrientationError.a < 0.0)
            {
                angularError = new Vector3(
                    -angularError.x,
                    -angularError.y,
                    -angularError.z);
            }

            return objectA.RotationMatrix * angularError;
        }

        public static Vector3 GetFixedAngularError(
            SoftShapePoint objectA,
            SoftShapePoint objectB)
        {
            Quaternion currentRelativeOrientation = objectB.RotationStatus.Inverse() *
                                                    objectA.RotationStatus;

            Quaternion relativeOrientationError = new Quaternion(1.0, 0.0, 0.0, 0.0) *
                                                  currentRelativeOrientation;

            var angularError = new Vector3(
                relativeOrientationError.b,
                relativeOrientationError.c,
                relativeOrientationError.d);

            if (relativeOrientationError.a < 0.0)
            {
                angularError = new Vector3(
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
			IShape simulationObjectA,
			IShape simulationObjectB,
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

		public static JacobianConstraint GetDOF(
			Vector3 linearComponentA,
			Vector3 linearComponentB,
			Vector3 angularComponentA,
			Vector3 angularComponentB,
            IShapeCommon simulationObjectA,
            IShapeCommon simulationObjectB,
			double constraintValue,
			double correctionValue,
			double cfm,
			double constraintLimit,
			ConstraintType type,
            	int? contactReference,
			StartImpulseProperties startImpulseProperties)
		{
			double jacobianVelocityValue = linearComponentA.Dot (simulationObjectA.LinearVelocity) +
			                               linearComponentB.Dot (simulationObjectB.LinearVelocity) +
			                               angularComponentA.Dot (simulationObjectA.AngularVelocity) +
			                               angularComponentB.Dot (simulationObjectB.AngularVelocity);

			jacobianVelocityValue -= constraintValue;

			if (startImpulseProperties == null)
				startImpulseProperties = new StartImpulseProperties(0.0);

			return new JacobianConstraint (
				simulationObjectA,
                simulationObjectB,
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

        public static JacobianConstraint GetDOF(
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            IShapeCommon simulationObjectA,
            IShapeCommon simulationObjectB,
            double constraintValue,
            double correctionValue,
            double cfm,
            double constraintLimit,
            ConstraintType type,
                int? contactReference,
            StartImpulseProperties startImpulseProperties)
        {
            double jacobianVelocityValue = angularComponentA.Dot(simulationObjectA.AngularVelocity) +
                                           angularComponentB.Dot(simulationObjectB.AngularVelocity);

            jacobianVelocityValue -= constraintValue;

            if (startImpulseProperties == null)
                startImpulseProperties = new StartImpulseProperties(0.0);

            return new JacobianConstraint(
                simulationObjectA,
                simulationObjectB,
                contactReference,
                new Vector3(),
                new Vector3(),
                angularComponentA,
                angularComponentB,
                type,
                jacobianVelocityValue,
                correctionValue,
                cfm,
                constraintLimit,
                startImpulseProperties);
        }

        public static JacobianConstraint GetDOF(
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            IShape simulationObjectA,
            IShape simulationObjectB,
            double constraintValue,
            double correctionValue,
            double cfm,
            double constraintLimit,
            ConstraintType type)
        {
            return GetDOF(
                angularComponentA,
                angularComponentB,
                simulationObjectA,
                simulationObjectB,
                constraintValue,
                correctionValue,
                cfm,
                constraintLimit,
                type,
                null,
                null);
        }

        public static JacobianConstraint GetDOF(
            Vector3 linearComponentA,
            Vector3 linearComponentB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            IShape simulationObjectA,
            IShape simulationObjectB,
            double constraintValue,
            double correctionValue,
            double cfm,
            double constraintLimit,
            ConstraintType type)
        {
            return GetDOF(
                linearComponentA,
                linearComponentB,
                angularComponentA,
                angularComponentB,
                simulationObjectA,
                simulationObjectB,
                constraintValue,
                correctionValue,
                cfm,
                constraintLimit,
                type,
                null,
                null);
        }

        public static JacobianConstraint GetDOF(
            Vector3 linearComponentA,
            Vector3 linearComponentB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            SoftShapePoint softShapePointA,
            SoftShapePoint softShapePointB,
            double constraintValue,
            double correctionValue,
            double cfm,
            double constraintLimit,
            ConstraintType type)
        {
            double jacobianVelocityValue = linearComponentA.Dot(softShapePointA.LinearVelocity) +
                                           linearComponentB.Dot(softShapePointB.LinearVelocity) +
                                           angularComponentA.Dot(softShapePointA.AngularVelocity) +
                                           angularComponentB.Dot(softShapePointB.AngularVelocity);

            jacobianVelocityValue -= constraintValue;
                        
            return new JacobianConstraint(
                softShapePointA,
                softShapePointB,
                linearComponentA,
                linearComponentB,
                angularComponentA,
                angularComponentB,
                type,
                jacobianVelocityValue,
                correctionValue,
                cfm,
                constraintLimit);
        }

        public static JacobianConstraint GetDOF(
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            SoftShapePoint softShapePointA,
            SoftShapePoint softShapePointB,
            double constraintValue,
            double correctionValue,
            double cfm,
            double constraintLimit,
            ConstraintType type)
        {
            double jacobianVelocityValue = angularComponentA.Dot(softShapePointA.AngularVelocity) +
                                           angularComponentB.Dot(softShapePointB.AngularVelocity);

            jacobianVelocityValue -= constraintValue;

            return new JacobianConstraint(
                softShapePointA,
                softShapePointB,
                angularComponentA,
                angularComponentB,
                type,
                jacobianVelocityValue,
                correctionValue,
                cfm,
                constraintLimit);
        }

        public static JacobianConstraint GetLinearLimit (
			IShape simulationObjectA, 
			IShape simulationObjectB, 
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

			return new JacobianConstraint ();
		}

		public static JacobianConstraint? GetAngularLimit (
			double angle,
			double K,
			double C,
			IShape simulationObjectA, 
			IShape simulationObjectB,
			Vector3 rotationAxis,
			double angularLimitMin,
			double angularLimitMax)
		{
			if (angle > angularLimitMax) {

				double angularLimit = 
					K *
					(angle - angularLimitMax);

				return GetDOF (
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

