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
			CollisionPointStructure[] collisionPointsStruct,
			SimulationObject[] simulationObjs,
			SimulationParameters simulationParameters)
		{
			var contactConstraints = new List<JacobianContact> ();

			for (int i = 0; i < collisionPointsStruct.Length; i++) 
			{
				CollisionPointStructure collisionPointStr = collisionPointsStruct [i];

				int indexA = collisionPointStr.ObjectA;
				int indexB = collisionPointStr.ObjectB;

				SimulationObject objectA = simulationObjs[indexA];
				SimulationObject objectB = simulationObjs[indexB];

				double restitutionCoefficient =
					(simulationObjs[indexA].RestitutionCoeff +
					 simulationObjs[indexB].RestitutionCoeff) * 0.5;

				double baumgarteStabilizationValue = 
					(simulationObjs[indexA].BaumgarteStabilizationCoeff +
					 simulationObjs[indexB].BaumgarteStabilizationCoeff) * 0.5;

                for (int h = 0; h < collisionPointStr.CollisionPointBase.Length; h++)
                {
                    for (int k = 0; k < collisionPointStr.CollisionPointBase[h].CollisionPoints.Length; k++)
                    {
                        Vector3 ra = collisionPointStr.CollisionPointBase[h].CollisionPoints[k].CollisionPointA - objectA.Position;
                        Vector3 rb = collisionPointStr.CollisionPointBase[h].CollisionPoints[k].CollisionPointB - objectB.Position;

                        Vector3 linearComponentA = (-1.0 * collisionPointStr.CollisionPointBase[h].CollisionPoints[k].CollisionNormal).Normalize();
                        Vector3 linearComponentB = -1.0 * linearComponentA;

                        Vector3 angularComponentA = ra.Cross(linearComponentA);
                        Vector3 angularComponentB = -1.0 * rb.Cross(linearComponentA);

                        Vector3 velocityA = objectA.LinearVelocity +
                                            objectA.AngularVelocity.Cross(ra);

                        Vector3 velocityB = objectB.LinearVelocity +
                                            objectB.AngularVelocity.Cross(rb);

                        Vector3 relativeVelocity = velocityB - velocityA;

                        if (relativeVelocity.Length() < 1E-12 &&
                            collisionPointStr.CollisionPointBase[h].Intersection &&
                            collisionPointStr.CollisionPointBase[h].ObjectDistance < 1E-10)
                            continue;

                        #region Normal direction contact

                        double linearComponent = linearComponentA.Dot(relativeVelocity);

                        double uCollision = restitutionCoefficient * Math.Max(0.0, linearComponent - simulationParameters.VelocityTolerance);

                        double correctionParameter = 0.0;

                        // Console.WriteLine("coll " + linearComponent);
                        if (collisionPointStr.CollisionPointBase[h].Intersection)
                        {
                            //Limit the Baum stabilization jitter effect 
                            correctionParameter = Math.Max(Math.Max(collisionPointStr.CollisionPointBase[h].ObjectDistance - simulationParameters.CompenetrationTolerance, 0.0) *
                                                    baumgarteStabilizationValue - uCollision, 0.0);
                        }

                        double correctedBounce = uCollision;

                        JacobianContact normalContact = JacobianCommon.GetDOF(
                            indexA,
                            indexB,
                            linearComponentA,
                            linearComponentB,
                            angularComponentA,
                            angularComponentB,
                            objectA,
                            objectB,
                            correctedBounce,
                            correctionParameter,
                            simulationParameters.NormalCFM,
                            0.0,
                            ConstraintType.Collision,
                            null,
                            collisionPointStr.CollisionPointBase[h].CollisionPoints[k].StartImpulseValue[0]);

                        #endregion

                        #region Friction Contact

                        JacobianContact[] frictionContact =
                            addFriction(
                                objectA,
                                objectB,
                                simulationParameters,
                                indexA,
                                indexB,
                                linearComponentA,
                                relativeVelocity,
                                ra,
                                rb,
                                collisionPointStr.CollisionPointBase[h].CollisionPoints[k].StartImpulseValue);

                        #endregion

                        contactConstraints.Add(normalContact);

                        int normalIndex = contactConstraints.Count - 1;
                        foreach (JacobianContact jc in frictionContact)
                        {
                            jc.SetContactReference(normalIndex);
                            contactConstraints.Add(jc);
                        }
                    }
                }
				
			}
			return contactConstraints;
		}

		#endregion

		#region Private Methods

		private static JacobianContact[] addFriction(
			SimulationObject objectA,
			SimulationObject objectB,
			SimulationParameters simulationParameters,
			int indexA,
			int indexB,
			Vector3 normal,
			Vector3 relativeVelocity,
			Vector3 ra,
			Vector3 rb,
			List<StartImpulseProperties> startImpulseProperties)
		{
			JacobianContact[] friction = new JacobianContact[2];

			var tx = new Vector3 ();
			var ty = new Vector3 ();

			GeometryUtilities.ComputeBasis(
				normal,
				ref tx,
				ref ty);

			double constraintLimit = 0.0;

            Vector3 tangentialVelocity = relativeVelocity -
                                         (normal.Dot(relativeVelocity)) *
                                         normal;

            #region Get start friction direction

            if (Vector3.Length (tangentialVelocity) >
				simulationParameters.ShiftToStaticFrictionTolerance) 
				constraintLimit = 0.5 * (objectA.DynamicFrictionCoeff + objectB.DynamicFrictionCoeff);
			else 
				constraintLimit = 0.5 * (objectA.StaticFrictionCoeff + objectB.StaticFrictionCoeff);
			
			#endregion

			#region Tangential Direction 1

			var linearComponentA = tx;
			var linearComponentB = -1.0 * linearComponentA;

			var angularComponentA = ra.Cross (linearComponentA);
			var angularComponentB = -1.0 * rb.Cross (linearComponentA);

			friction [0] = JacobianCommon.GetDOF (
				indexA,
				indexB,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				objectA,
				objectB,
				0.0,
				0.0,
				simulationParameters.FrictionCFM,
				constraintLimit,
				ConstraintType.Friction,
                	null,
				startImpulseProperties[1]);

			#endregion

			#region Tangential Direction 2

			linearComponentA = ty;
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
				objectA,
				objectB,
				0.0,
				0.0,
				simulationParameters.FrictionCFM,
				constraintLimit,
				ConstraintType.Friction,
                null,
				startImpulseProperties[2]);

			#endregion

			return friction;
		}

		#endregion

	}
}

