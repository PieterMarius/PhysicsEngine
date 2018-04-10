/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using System;
using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;
using SharpEngineMathUtility;
using SharpPhysicsEngine.CollisionEngine;
using System.Linq;

namespace SharpPhysicsEngine
{
    internal sealed class ContactConstraintBuilder
	{
        #region Fields

        //Box Constraints
        private const int frictionDirections = 2;
        private readonly PhysicsEngineParameters simulationParameters;

        public int ConstraintsNumber { get { return 3; } } 

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
            double timeStep,
            IShape objectA,
            IShape objectB)
        {
            var contactConstraints = new List<JacobianConstraint>();

            if (objectA is ISoftShape && 
                !(objectB is SoftShape))
            {
                contactConstraints.AddRange(BuildSoftBodyVSRigidBodyCollisionConstraints(collisionPointStr, (ISoftShape)objectA, objectB, timeStep));
            }
            else if (objectB is ISoftShape && 
                    !(objectA is SoftShape))
            {
                contactConstraints.AddRange(BuildSoftBodyVSRigidBodyCollisionConstraints(collisionPointStr, objectA, (ISoftShape)objectB, timeStep));
            }
            else
            {
                contactConstraints.AddRange(BuildRigidBodyCollisionConstraints(collisionPointStr, objectA, objectB, timeStep));
            }

            return contactConstraints;
        }

        #endregion

        #region Private Methods
        
        private List<JacobianConstraint> BuildSoftBodyVSRigidBodyCollisionConstraints(
            CollisionPointStructure collisionPointStr,
            ISoftShape objectA,
            IShape objectB,
            double timeStep)
        {
            List<JacobianConstraint> contactConstraints = new List<JacobianConstraint>();

            IShape iSoftShape = (IShape)objectA;

            double restitutionCoeff = GetRestitutionCoeff(iSoftShape, objectB);

            double baumgarteStabValue = GetBaumgarteStabilizationValue(iSoftShape, objectB, timeStep);
                                    
            for (int h = 0; h < collisionPointStr.CollisionPointBase.Length; h++)
            {
                int?[] linkedID = collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointA.LinkedID.Distinct().ToArray();

                double distanceSum = GetSoftBodyPointDistanceSum(
                    collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointA.Vertex,
                    objectA,
                    linkedID);

                for (int i = 0; i < linkedID.Length; i++)
                {
                    SoftShapePoint softShapePoint = objectA.ShapePoints.FirstOrDefault(x => x.ID == linkedID[i]);

                    if (softShapePoint == null)
                        continue;

                    double distanceWeigth = 1.0;

                    if (distanceSum > 0.0)
                        distanceWeigth = Vector3.Length(softShapePoint.Position - collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointA.Vertex) / distanceSum;
                    if (distanceWeigth < 1E-10)
                        continue;

                    Vector3 ra = Vector3.Zero();
                    Vector3 rb = collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointB.Vertex - objectB.Position;

                    ////Component
                    Vector3 linearComponentA = (-1.0 * collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionNormal).Normalize();
                    Vector3 linearComponentB = -1.0 * linearComponentA;

                    Vector3 angularComponentA = Vector3.Zero();
                    Vector3 angularComponentB = -1.0 * rb.Cross(linearComponentA);

                    ////Velocity
                    Vector3 velocityA = softShapePoint.LinearVelocity;

                    Vector3 velocityB = objectB.LinearVelocity +
                                        objectB.AngularVelocity.Cross(rb);

                    Vector3 relativeVelocity = velocityB - velocityA;


                    if (relativeVelocity.Length() < 1E-12 &&
                        collisionPointStr.CollisionPointBase[h].CollisionPoint.Intersection &&
                        collisionPointStr.CollisionPointBase[h].CollisionPoint.Distance < 1E-10)
                        continue;

                    #region Normal direction contact

                    double linearComponent = linearComponentA.Dot(relativeVelocity);

                    double uCollision = restitutionCoeff * Math.Max(0.0, linearComponent - simulationParameters.VelocityTolerance);

                    double correctionParameter = 0.0;

                    if (collisionPointStr.CollisionPointBase[h].CollisionPoint.Intersection)
                    {
                        //Limit the Baum stabilization jitter effect 
                        correctionParameter = Math.Max(Math.Max(collisionPointStr.CollisionPointBase[h].CollisionPoint.Distance - simulationParameters.CompenetrationTolerance, 0.0) *
                                                baumgarteStabValue - uCollision, 0.0);
                    }

                    double correctedBounce = uCollision * distanceWeigth;
                    correctionParameter = correctionParameter * distanceWeigth;

                    JacobianConstraint normalContact = JacobianCommon.GetDOF(
                        linearComponentA,
                        linearComponentB,
                        angularComponentA,
                        angularComponentB,
                        softShapePoint,
                        objectB,
                        correctedBounce,
                        correctionParameter,
                        simulationParameters.NormalCFM,
                        0.0,
                        ConstraintType.Collision,
                        null,
                        null);

                    #endregion

                    #region Friction Contact

                    JacobianConstraint[] frictionContact =
                        AddFrictionConstraints(
                            iSoftShape,
                            objectB,
                            simulationParameters,
                            linearComponentA,
                            relativeVelocity,
                            ra,
                            rb,
                            null,
                            softShapePoint);

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

        private List<JacobianConstraint> BuildSoftBodyVSRigidBodyCollisionConstraints(
            CollisionPointStructure collisionPointStr,
            IShape objectA,
            ISoftShape objectB,
            double timeStep)
        {
            List<JacobianConstraint> contactConstraints = new List<JacobianConstraint>();

            IShape iSoftShape = (IShape)objectB;

            double restitutionCoeff = GetRestitutionCoeff(iSoftShape, objectA);

            double baumgarteStabValue = GetBaumgarteStabilizationValue(iSoftShape, objectA, timeStep);
            
            for (int h = 0; h < collisionPointStr.CollisionPointBase.Length; h++)
            {
                int?[] linkedID = collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointB.LinkedID.Distinct().ToArray();

                double distanceSum = GetSoftBodyPointDistanceSum(
                    collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointB.Vertex,
                    objectB,
                    linkedID);

                for (int i = 0; i < linkedID.Length; i++)
                {
                    SoftShapePoint softShapePoint = objectB.ShapePoints.FirstOrDefault(x => x.ID == linkedID[i]);

                    if (softShapePoint == null)
                        continue;

                    double distanceWeigth = 1.0;

                    if (distanceSum > 0.0)
                        distanceWeigth = Vector3.Length(softShapePoint.Position - collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointB.Vertex) / distanceSum;
                    if (distanceWeigth < 1E-10)
                        continue;

                    Vector3 ra = collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionPointA.Vertex - objectA.Position;
                    Vector3 rb = Vector3.Zero();
                    
                    ////Component
                    Vector3 linearComponentA = (-1.0 * collisionPointStr.CollisionPointBase[h].CollisionPoint.CollisionNormal).Normalize();
                    Vector3 linearComponentB = -1.0 * linearComponentA;

                    Vector3 angularComponentA = ra.Cross(linearComponentA);
                    Vector3 angularComponentB = Vector3.Zero();

                    ////Velocity
                    
                    Vector3 velocityA = objectA.LinearVelocity +
                                        objectA.AngularVelocity.Cross(ra);

                    Vector3 velocityB = softShapePoint.LinearVelocity;

                    Vector3 relativeVelocity = velocityB - velocityA;
                    
                    if (relativeVelocity.Length() < 1E-12 &&
                        collisionPointStr.CollisionPointBase[h].CollisionPoint.Intersection &&
                        collisionPointStr.CollisionPointBase[h].CollisionPoint.Distance < 1E-10)
                        continue;

                    #region Normal direction contact

                    double linearComponent = linearComponentA.Dot(relativeVelocity);

                    double uCollision = restitutionCoeff * Math.Max(0.0, linearComponent - simulationParameters.VelocityTolerance);

                    double correctionParameter = 0.0;

                    if (collisionPointStr.CollisionPointBase[h].CollisionPoint.Intersection)
                    {
                        //Limit the Baum stabilization jitter effect 
                        correctionParameter = Math.Max(Math.Max(collisionPointStr.CollisionPointBase[h].CollisionPoint.Distance - simulationParameters.CompenetrationTolerance, 0.0) *
                                                baumgarteStabValue - uCollision, 0.0);
                    }

                    double correctedBounce = uCollision * distanceWeigth;
                    correctionParameter = correctionParameter * distanceWeigth;

                    JacobianConstraint normalContact = JacobianCommon.GetDOF(
                        linearComponentA,
                        linearComponentB,
                        angularComponentA,
                        angularComponentB,
                        objectA,
                        softShapePoint,
                        correctedBounce,
                        correctionParameter,
                        simulationParameters.NormalCFM,
                        0.0,
                        ConstraintType.Collision,
                        null,
                        null//collisionPointStr.CollisionPointBase[h].CollisionPoint.StartImpulseValue[0]
                        );

                    #endregion

                    #region Friction Contact

                    JacobianConstraint[] frictionContact =
                        AddFrictionConstraints(
                            objectA,
                            iSoftShape,
                            simulationParameters,
                            linearComponentA,
                            relativeVelocity,
                            ra,
                            rb,
                            null,//collisionPointStr.CollisionPointBase[h].CollisionPoint.StartImpulseValue,
                            softShapePoint);

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
         
        private static double GetSoftBodyPointDistanceSum(
            Vector3 collisionPoint,
            ISoftShape softShape, 
            int?[] linkedID)
        {
            double distanceSum = 0.0;

            for (int i = 0; i < linkedID.Length; i++)
            {
                SoftShapePoint softShapePoint = softShape.ShapePoints.FirstOrDefault(x => x.ID == linkedID[i]);

                if (softShapePoint == null)
                    continue;

                distanceSum += Vector3.Length(softShapePoint.Position - collisionPoint);
            }

            return distanceSum;
        }

        private double GetRestitutionCoeff(
            IShape softShape,
            IShape shape)
        {
            return (softShape.RestitutionCoeff +
                   shape.RestitutionCoeff) * 0.5;
        }

        private double GetBaumgarteStabilizationValue(
            IShape softShape,
            IShape shape,
            double timeStep)
        {
            double baumgarteStabilizationValue =
                    (softShape.RestoreCoeff +
                     shape.RestoreCoeff) * 0.5;

            return baumgarteStabilizationValue / timeStep;
        }

        private List<JacobianConstraint> BuildRigidBodyCollisionConstraints(
			CollisionPointStructure collisionPointStr,
            IShape objectA,
			IShape objectB,
            double timeStep)

        {
			List<JacobianConstraint> contactConstraints = new List<JacobianConstraint>();

			double restitutionCoefficient =
					(objectA.RestitutionCoeff +
					 objectB.RestitutionCoeff) * 0.5;

			double baumgarteStabilizationValue =
				(objectA.RestoreCoeff +
				 objectB.RestoreCoeff) * 0.5;

            baumgarteStabilizationValue = baumgarteStabilizationValue / timeStep;

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
						collisionPoint.Intersection &&
						collisionPoint.Distance < 1E-10)
						continue;

					#region Normal direction contact

					double linearComponent = linearComponentA.Dot(relativeVelocity);

					double uCollision = restitutionCoefficient * Math.Max(0.0, linearComponent - simulationParameters.VelocityTolerance);

					double correctionParameter = 0.0;

					if (collisionPoint.Intersection)
					{
						//Limit the Baum stabilization jitter effect 
						correctionParameter = Math.Max(Math.Max(collisionPoint.Distance- simulationParameters.CompenetrationTolerance, 0.0) *
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
                        (collisionPoint.StartImpulseValue.Count > 0) ? collisionPoint.StartImpulseValue[0] : null);

                    #endregion

                    contactConstraints.Add(normalContact);

                    #region Friction Contact

                    JacobianConstraint[] frictionContact =
                        AddFrictionConstraints(
                            objectA,
                            objectB,
                            simulationParameters,
                            linearComponentA,
                            relativeVelocity,
                            ra,
                            rb,
                            (collisionPoint.StartImpulseValue.Count > 0) ? collisionPoint.StartImpulseValue.ToArray() : null);

                    #endregion

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

		private JacobianConstraint[] AddFrictionConstraints(
			IShape objectA,
			IShape objectB,
			PhysicsEngineParameters simulationParameters,
			Vector3 normal,
			Vector3 relativeVelocity,
			Vector3 ra,
			Vector3 rb,
			StartImpulseProperties[] startImpulseProperties,
            SoftShapePoint softShapePoint)
		{
            JacobianConstraint[] friction = new JacobianConstraint[frictionDirections];

            Vector3[] frictionDirection = GetFrictionCone(normal, frictionDirections);

            double constraintLimit = 0.0;

            Vector3 tangentialVelocity = relativeVelocity -
                                         normal.Dot(relativeVelocity) *
                                         normal;

            if (Vector3.Length(tangentialVelocity) > simulationParameters.ShiftToStaticFrictionTolerance)
                constraintLimit = 0.5 * (objectA.DynamicFrictionCoeff + objectB.DynamicFrictionCoeff);
            else
                constraintLimit = 0.5 * (objectA.StaticFrictionCoeff + objectB.StaticFrictionCoeff);

            for (int i = 0; i < frictionDirections; i++)
            {
                var linearComponentA = frictionDirection[i];
                var linearComponentB = -1.0 * linearComponentA;

                var angularComponentA = ra.Cross(linearComponentA);
                var angularComponentB = -1.0 * rb.Cross(linearComponentA);

                friction[i] = JacobianCommon.GetDOF(
                    linearComponentA,
                    linearComponentB,
                    angularComponentA,
                    angularComponentB,
                    (objectA is ISoftShape) ? (IShapeCommon)softShapePoint: objectA,
                    (objectB is ISoftShape) ? (IShapeCommon)softShapePoint: objectB,
                    0.0,
                    0.0,
                    simulationParameters.FrictionCFM,
                    constraintLimit,
                    ConstraintType.Friction,
                    null,
                    startImpulseProperties?[i]);
            }

            return friction;
		}

        private JacobianConstraint[] AddFrictionConstraints(
            IShape objectA,
            IShape objectB,
            PhysicsEngineParameters simulationParameters,
            Vector3 normal,
            Vector3 relativeVelocity,
            Vector3 ra,
            Vector3 rb,
            StartImpulseProperties[] startImpulseProperties)
        {
            JacobianConstraint[] friction = new JacobianConstraint[frictionDirections];

            Vector3[] frictionDirection = GetFrictionCone(normal, frictionDirections);
            
            double constraintLimit = 0.0;

            Vector3 tangentialVelocity = relativeVelocity -
                                         normal.Dot(relativeVelocity) *
                                         normal;
            
            if (Vector3.Length(tangentialVelocity) > simulationParameters.ShiftToStaticFrictionTolerance)
                constraintLimit = 0.5 * (objectA.DynamicFrictionCoeff + objectB.DynamicFrictionCoeff);
            else
                constraintLimit = 0.5 * (objectA.StaticFrictionCoeff + objectB.StaticFrictionCoeff);

            for (int i = 0; i < frictionDirections; i++)
            {
                var linearComponentA = frictionDirection[i];
                var linearComponentB = -1.0 * linearComponentA;

                var angularComponentA = ra.Cross(linearComponentA);
                var angularComponentB = -1.0 * rb.Cross(linearComponentA);

                friction[i] = JacobianCommon.GetDOF(
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
                    startImpulseProperties?[i + 1]);
            }
            
            return friction;
        }

        private Vector3[] GetFrictionCone(Vector3 normal, int nDirection)
        {
            var coneDirection = new Vector3[nDirection];

            var tx = new Vector3();
            var ty = new Vector3();

            GeometryUtilities.ComputeBasis(
                normal,
                ref tx,
                ref ty);

            coneDirection[0] = tx;
            coneDirection[1] = ty;

            if (nDirection > 2)
            {
                double step = Math.PI / nDirection;
                for (int i = 0; i < nDirection; i++)
                    coneDirection[i] = Matrix3x3.GetRotationMatrix(normal, step * i) * tx;
            }
                       
            return coneDirection;
        }

        #endregion

    }
}

