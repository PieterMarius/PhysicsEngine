using System;
using System.Collections.Generic;
using CollisionEngine;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public static class ContactConstraint
	{
		#region Public Methods

		public static List<JacobianContact> BuildJoints(
			List<CollisionPointStructure> collisionPointsStruct,
			SimulationObject[] simulationObjs,
			SimulationParameters simulationParameters)
		{
			List<JacobianContact> contactConstraints = new List<JacobianContact> ();

			for (int i = 0; i < collisionPointsStruct.Count; i++) 
			{
				CollisionPointStructure collisionPointStr = collisionPointsStruct [i];

				int indexA = collisionPointStr.ObjectA;
				int indexB = collisionPointStr.ObjectB;

				double restitutionCoefficient =
					(simulationObjs [indexA].RestitutionCoeff +
						simulationObjs [indexB].RestitutionCoeff) * 0.5;

				for (int k = 0; k < collisionPointStr.CollisionPoints.Length; k++) 
				{
					Vector3 collisionPoint;

					if (collisionPointStr.Intersection)
						collisionPoint = collisionPointStr.CollisionPoints [k].collisionPointA;
					else
						collisionPoint = (collisionPointStr.CollisionPoints [k].collisionPointA +
							collisionPointStr.CollisionPoints [k].collisionPointB) * 0.5;

					Vector3 ra = collisionPoint - simulationObjs [indexA].Position;
					Vector3 rb = collisionPoint - simulationObjs [indexB].Position;

					Vector3 linearComponentA = (-1.0 * collisionPointStr.CollisionPoints [k].collisionNormal).Normalize ();
					Vector3 linearComponentB = -1.0 * linearComponentA;

					Vector3 angularComponentA = ra.Cross (linearComponentA);
					Vector3 angularComponentB = -1.0 * rb.Cross (linearComponentA);

					Vector3 velocityA = simulationObjs [indexA].LinearVelocity +
						simulationObjs [indexA].AngularVelocity.Cross (ra);

					Vector3 velocityB = simulationObjs [indexB].LinearVelocity +
						simulationObjs [indexB].AngularVelocity.Cross (rb);

					Vector3 relativeVelocity = velocityB - velocityA;

					Vector3 tangentialVelocity = relativeVelocity -
						(linearComponentA.Dot (relativeVelocity)) * linearComponentA;

					#region Normal direction contact

					double correctionParameter = (collisionPointStr.Intersection) ? 
						collisionPointStr.ObjectDistance * simulationParameters.BaumStabilization :
						0.0;

					double correctedBounce = linearComponentA.Dot (relativeVelocity) * restitutionCoefficient + correctionParameter;

					JacobianContact normalContact = JacobianCommon.GetDOF (
						indexA,
						indexB,
						linearComponentA,
						linearComponentB,
						angularComponentA,
						angularComponentB,
						simulationObjs [indexA],
						simulationObjs [indexB],
						correctedBounce,
						ConstraintType.Collision);

					#endregion

					#region Friction Contact

					JacobianContact[] frictionContact = 
						addFriction (
							simulationObjs,
							simulationParameters,
							indexA,
							indexB,
							linearComponentA,
							tangentialVelocity,
							ra,
							rb);

					#endregion

					contactConstraints.Add (normalContact);
					contactConstraints.Add (frictionContact[0]);
					contactConstraints.Add (frictionContact[1]);
				}
			}
			return contactConstraints;
		}

		#endregion

		#region Private Methods

		private static JacobianContact[] addFriction(
			SimulationObject[] simulationObjects,
			SimulationParameters simulationParameters,
			int indexA,
			int indexB,
			Vector3 normal,
			Vector3 tangentialVelocity,
			Vector3 ra,
			Vector3 rb)
		{
			JacobianContact[] friction = new JacobianContact[2];

			Vector3 linearComponentA = new Vector3 ();
			Vector3 linearComponentB = new Vector3 ();
			Vector3 angularComponentA = new Vector3 ();
			Vector3 angularComponentB = new Vector3 ();

			Vector3 t = new Vector3 ();

			double constraintLimit = 0.0;

			#region Get start friction direction

			if (Vector3.Length (tangentialVelocity) >
				simulationParameters.ShiftToStaticFrictionTolerance) 
			{
				constraintLimit = 0.5 *
					(simulationObjects [indexA].DynamicFrictionCoeff +
						simulationObjects [indexB].DynamicFrictionCoeff);

				t = tangentialVelocity.Normalize ();
			} 
			else 
			{
				constraintLimit = 0.5 *
					(simulationObjects [indexA].StaticFrictionCoeff +
						simulationObjects [indexB].StaticFrictionCoeff);

				t = GeometryUtilities.ProjectVectorOnPlane (normal);
			}

			#endregion

			#region Tangential Direction 1

			linearComponentA = t;
			linearComponentB = -1.0 * linearComponentA;

			angularComponentA = ra.Cross (linearComponentA);
			angularComponentB = -1.0 * rb.Cross (linearComponentA);

			friction [0] = JacobianCommon.GetDOF (
				indexA,
				indexB,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				simulationObjects [indexA],
				simulationObjects [indexB],
				constraintLimit,
				ConstraintType.Friction,
				-1);

			#endregion

			#region Tangential Direction 2

			linearComponentA = t.Cross (normal).Normalize ();
			linearComponentB = -1.0 * linearComponentA;

			angularComponentA = ra.Cross (linearComponentA);
			angularComponentB = -1.0 * rb.Cross (linearComponentA);

			friction [1] = JacobianCommon.GetDOF (
				indexA,
				indexB,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				simulationObjects [indexA],
				simulationObjects [indexB],
				constraintLimit,
				ConstraintType.Friction,
				-2);

			#endregion

			return friction;
		}

		#endregion

	}
}

