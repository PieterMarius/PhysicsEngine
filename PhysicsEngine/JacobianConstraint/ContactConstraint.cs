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
			var contactConstraints = new List<JacobianContact> ();

			for (int i = 0; i < collisionPointsStruct.Count; i++) 
			{
				CollisionPointStructure collisionPointStr = collisionPointsStruct [i];

				int indexA = collisionPointStr.ObjectA;
				int indexB = collisionPointStr.ObjectB;

				double restitutionCoefficient =
					(simulationObjs[indexA].RestitutionCoeff +
					 simulationObjs[indexB].RestitutionCoeff) * 0.5;

				for (int k = 0; k < collisionPointStr.CollisionPoints.Length; k++) 
				{
					Vector3 collisionPoint;

					if (collisionPointStr.Intersection)
						collisionPoint = collisionPointStr.CollisionPoints [k].CollisionPointA;
					else
						collisionPoint = (collisionPointStr.CollisionPoints [k].CollisionPointA +
							collisionPointStr.CollisionPoints [k].CollisionPointB) * 0.5;

					Vector3 ra = collisionPoint - simulationObjs [indexA].Position;
					Vector3 rb = collisionPoint - simulationObjs [indexB].Position;

					Vector3 linearComponentA = (-1.0 * collisionPointStr.CollisionPoints [k].CollisionNormal).Normalize ();
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
								Math.Max(collisionPointStr.ObjectDistance - simulationParameters.CompenetrationTolerance, 0.0) *
									simulationParameters.BaumStabilization
								:
								0.0;

					double linearComponent = linearComponentA.Dot(relativeVelocity);

					double uCollision = restitutionCoefficient * Math.Max(0.0, linearComponent);

					//Limit the Baum stabilization jitter effect
					correctionParameter = Math.Min(Math.Max(correctionParameter - uCollision, 0.0), simulationParameters.MaxCorrectionValue);

					double correctedBounce = uCollision +
											 correctionParameter;

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
						simulationParameters.NormalCFM,
						0.0,
						ConstraintType.Collision,
						null,
						collisionPointStr.CollisionPoints[k].StartImpulseValue[0]);

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
							rb,
							collisionPointStr.CollisionPoints[k].StartImpulseValue);

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
			Vector3 rb,
			List<StartImpulseProperties> startImpulseProperties)
		{
			JacobianContact[] friction = new JacobianContact[2];

			var linearComponentA = new Vector3 ();
			var linearComponentB = new Vector3 ();
			var angularComponentA = new Vector3 ();
			var angularComponentB = new Vector3 ();

			var t = new Vector3 ();

			double constraintLimit = 0.0;

			#region Get start friction direction

			if (Vector3.Length (tangentialVelocity) >
				simulationParameters.ShiftToStaticFrictionTolerance) 
			{
				constraintLimit = 0.5 * (simulationObjects [indexA].DynamicFrictionCoeff + simulationObjects [indexB].DynamicFrictionCoeff);

				t = tangentialVelocity.Normalize ();
			} 
			else 
			{
				constraintLimit = 0.5 * (simulationObjects[indexA].StaticFrictionCoeff + simulationObjects[indexB].StaticFrictionCoeff);

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
				0.0,
				simulationParameters.FrictionCFM,
				constraintLimit,
				ConstraintType.Friction,
				-1,
				startImpulseProperties[1]);

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
				0.0,
				simulationParameters.FrictionCFM,
				constraintLimit,
				ConstraintType.Friction,
				-2,
				startImpulseProperties[2]);

			#endregion

			return friction;
		}

		#endregion

	}
}

