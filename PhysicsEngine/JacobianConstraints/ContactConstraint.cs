using System;
using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;
using SharpPhysicsEngine.CollisionEngine;
using System.Linq;

namespace SharpPhysicsEngine
{
	public sealed class ContactConstraintBuilder
	{
		#region Fields

		private readonly PhysicsEngineParameters simulationParameters;

		#endregion Fields


		#region Constructor

		public ContactConstraintBuilder(
			PhysicsEngineParameters simulationParameters)
		{
			this.simulationParameters = simulationParameters;
		}

		#endregion Constructor


		#region Public Methods

        public List<JacobianConstraint> BuildJoints(
            CollisionPointStructure collisionPointStr,
            IShape objectA,
            IShape objectB)
        {
            var contactConstraints = new List<JacobianConstraint>();

            if (objectA is ISoftShape && !(objectB is SoftShape))
            {
                contactConstraints.AddRange(BuildSoftBodyVSRigidBodyCollisionJoints(collisionPointStr, (ISoftShape)objectA, objectB, 0));
            }
            else if (objectB is ISoftShape && !(objectA is SoftShape))
            {
                contactConstraints.AddRange(BuildSoftBodyVSRigidBodyCollisionJoints(collisionPointStr, (ISoftShape)objectB, objectA, 1));
            }
            else
            {
                contactConstraints.AddRange(BuildRigidBodyCollisionJoints(collisionPointStr, objectA, objectB));
            }

            return contactConstraints;
        }
        
		#endregion

		#region Private Methods

		private List<JacobianConstraint> BuildSoftBodyVSRigidBodyCollisionJoints(
            CollisionPointStructure collisionPointStr,
            ISoftShape softShape,
            IShape rigidShape,
            int collisionIndex)
		{
			List<JacobianConstraint> contactConstraints = new List<JacobianConstraint>();

            IShape iSoftShape = (IShape)softShape;

            double restitutionCoefficient =
                    (iSoftShape.RestitutionCoeff +
                     rigidShape.RestitutionCoeff) * 0.5;

            double baumgarteStabilizationValue =
                (iSoftShape.RestoreCoeff +
                 rigidShape.RestoreCoeff) * 0.5;

            double normalDirection = 1.0;
            
            if (collisionIndex == 1)
                normalDirection = -1.0;

            for (int h = 0; h < collisionPointStr.CollisionPointBase.Length; h++)
            {
                int?[] linkedID = collisionPointStr.CollisionPointBase[h].CollisionPoint.GetCollisionVertex(collisionIndex).LinkedID.Distinct().ToArray();
                
                for (int i = 0; i < linkedID.Length; i++)
                {
                    SoftShapePoint softShapePoint = softShape.ShapePoints.First(x => x.ID == linkedID[i]);

                    Vector3 collisionVertex = softShapePoint.Position;

                    Vector3 r_softShape = new Vector3();                    
                    Vector3 r_rigidShape = collisionVertex - rigidShape.Position;

                    ////Component

                    Vector3 linearComponentSoftShape = (normalDirection * collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionNormal).Normalize();
                    Vector3 linearComponentRigidShape = -1.0 * linearComponentSoftShape;

                    Vector3 angularComponentRigidShape = -1.0 * r_rigidShape.Cross(linearComponentSoftShape);
                    
                    ////Velocity
                    Vector3 softShapeVelocity = softShapePoint.LinearVelocity;

                    Vector3 rigidShapeVelocity = rigidShape.LinearVelocity +
                                                 rigidShape.AngularVelocity.Cross(r_rigidShape);

                    Vector3 relativeVelocity = softShapeVelocity - rigidShapeVelocity;


                    if (relativeVelocity.Length() < 1E-12 &&
                        collisionPointStr.CollisionPointBase[h].Intersection &&
                        collisionPointStr.CollisionPointBase[h].ObjectDistance < 1E-10)
                        continue;

                    #region Normal direction contact

                    double linearComponent = linearComponentSoftShape.Dot(relativeVelocity);

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

                    JacobianConstraint normalContact = JacobianCommon.GetDOF(
                        linearComponentSoftShape,
                        linearComponentRigidShape,
                        r_softShape,
                        angularComponentRigidShape,
                        (collisionIndex == 0) ? (IShapeCommon)softShapePoint : rigidShape,
                        (collisionIndex == 0) ? rigidShape: (IShapeCommon)softShapePoint,
                        correctedBounce,
                        correctionParameter,
                        simulationParameters.NormalCFM,
                        0.0,
                        ConstraintType.Collision,
                        null,
                        collisionPointStr.CollisionPointBase[h].CollisionPoint.StartImpulseValue[0]);

                    #endregion

                    #region Friction Contact

                    JacobianConstraint[] frictionContact =
                        addFriction(
                            (collisionIndex == 0) ? iSoftShape : rigidShape,
                            (collisionIndex == 0) ? rigidShape : iSoftShape,
                            simulationParameters,
                            linearComponentSoftShape,
                            relativeVelocity,
                            r_softShape,
                            r_rigidShape,
                            collisionPointStr.CollisionPointBase[h].CollisionPoint.StartImpulseValue,
                            softShapePoint,
                            (collisionIndex == 0) ? linkedID[i]: null,
                            (collisionIndex == 1) ? linkedID[i]: null);

                    #endregion

                    contactConstraints.Add(normalContact);

                    int normalIndex = contactConstraints.Count - 1;
                    foreach (JacobianConstraint fjc in frictionContact)
                    {
                        fjc.SetContactReference(normalIndex);
                        contactConstraints.Add(fjc);
                    }

                }
            }
            
            return contactConstraints;
		}

		private List<JacobianConstraint> BuildRigidBodyCollisionJoints(
			CollisionPointStructure collisionPointStr,
			IShape objectA,
			IShape objectB)
		{
			List<JacobianConstraint> contactConstraints = new List<JacobianConstraint>();

			double restitutionCoefficient =
					(objectA.RestitutionCoeff +
					 objectB.RestitutionCoeff) * 0.5;

			double baumgarteStabilizationValue =
				(objectA.RestoreCoeff +
				 objectB.RestoreCoeff) * 0.5;

			for (int h = 0; h < collisionPointStr.CollisionPointBase.Length; h++)
			{
				for (int k = 0; k < collisionPointStr.CollisionPointBase[h].CollisionPoints.Length; k++)
				{
					CollisionPoint collisionPoint = collisionPointStr.CollisionPointBase[h].CollisionPoints[k];

					Vector3 ra = collisionPoint.CollisionPointA.Vertex - objectA.Position;
					Vector3 rb = collisionPoint.CollisionPointB.Vertex - objectB.Position;

					Vector3 linearComponentA = (-1.0 * collisionPoint.CollisionNormal).Normalize();
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

					JacobianConstraint normalContact = JacobianCommon.GetDOF(
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
						collisionPoint.StartImpulseValue[0]);

					#endregion

					#region Friction Contact

					JacobianConstraint[] frictionContact =
						addFriction(
							objectA,
							objectB,
							simulationParameters,
							linearComponentA,
							relativeVelocity,
							ra,
							rb,
							collisionPoint.StartImpulseValue,
                            null,
                            null,
                            null);

					#endregion

					contactConstraints.Add(normalContact);

					int normalIndex = contactConstraints.Count - 1;
					foreach (JacobianConstraint fjc in frictionContact)
					{
						fjc.SetContactReference(normalIndex);
						contactConstraints.Add(fjc);
					}
				}
			}

			return contactConstraints;
		}

		private JacobianConstraint[] addFriction(
			IShape objectA,
			IShape objectB,
			PhysicsEngineParameters simulationParameters,
			Vector3 normal,
			Vector3 relativeVelocity,
			Vector3 ra,
			Vector3 rb,
			List<StartImpulseProperties> startImpulseProperties,
            SoftShapePoint softShapePoint,
            int? softShapePointIndexA,
            int? softShapePointIndexB)
		{
			JacobianConstraint[] friction = new JacobianConstraint[2];

			var tx = new Vector3 ();
			var ty = new Vector3 ();

			GeometryUtilities.ComputeBasis(
				normal,
				ref tx,
				ref ty);

			double constraintLimit = 0.0;

			Vector3 tangentialVelocity = relativeVelocity -
										 normal.Dot(relativeVelocity) *
										 normal;

			#region Get start friction direction

			if (Vector3.Length (tangentialVelocity) > simulationParameters.ShiftToStaticFrictionTolerance) 
				constraintLimit = 0.5 * (objectA.DynamicFrictionCoeff + objectB.DynamicFrictionCoeff);
			else 
				constraintLimit = 0.5 * (objectA.StaticFrictionCoeff + objectB.StaticFrictionCoeff);
			
			#endregion

			#region Tangential Direction 1

			var linearComponentA = tx;
			var linearComponentB = -1.0 * linearComponentA;

			var angularComponentA = ra.Cross (linearComponentA);
			var angularComponentB = -1.0 * rb.Cross (linearComponentA);

            friction[0] = JacobianCommon.GetDOF(
                linearComponentA,
                linearComponentB,
                angularComponentA,
                angularComponentB,
                (softShapePointIndexA.HasValue) ? (IShapeCommon)softShapePoint : objectA,
                (softShapePointIndexB.HasValue) ? (IShapeCommon)softShapePoint : objectB,
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
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
                (softShapePointIndexA.HasValue) ? (IShapeCommon)softShapePoint : objectA,
                (softShapePointIndexB.HasValue) ? (IShapeCommon)softShapePoint : objectB,
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

