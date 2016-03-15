﻿using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using CollisionEngine;
using SimulationObjectDefinition;


namespace MonoPhysicsEngine
{
	public class JacobianConstraintBuilder: IJacobianConstraintBuilder
	{

		#region Constructor

		public JacobianConstraintBuilder () {}

		#endregion

		#region Public Methods

		public List<JacobianContact> GetJacobianConstraint(
			List<CollisionPointStructure> collisionPointsStruct,
			List<SimulationJoint> simulationJointList,
			SimulationObject[] simulationObjs,
			SimulationParameters simulationParameters)
		{
			List<JacobianContact> constraint = new List<JacobianContact> ();

			#region Collision Contact

			constraint.AddRange (
				this.BuildContactJoints (
					collisionPointsStruct,
					simulationObjs,
					simulationParameters));

			#endregion

			#region Joint

			foreach(SimulationJoint simJoint in simulationJointList)
			{
				foreach (Joint joint in simJoint.JointList) 
				{
					switch (joint.Type) 
					{
					case JointType.Fixed:
						constraint.AddRange (
							this.BuildFixedJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
							break;

					case JointType.Slider:
						constraint.AddRange (
							this.BuildSliderJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Hinge:
						constraint.AddRange (
							this.BuildSliderJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;
					}
				}
			}

			#endregion

			return constraint;
		}

		#endregion

		#region Private Methods

		#region Build Joints

		private List<JacobianContact> BuildContactJoints(
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
					1.0 + (simulationObjs [indexA].RestitutionCoeff +
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

					Vector3 collisionNormal = (collisionPointStr.CollisionPoints [k].collisionNormal * -1.0).Normalize ();
					Vector3 negCollisionNormal = -1.0 * collisionNormal;

					Vector3 velocityA = simulationObjs [indexA].LinearVelocity +
					                    simulationObjs [indexA].AngularVelocity.Cross (ra);

					Vector3 velocityB = simulationObjs [indexB].LinearVelocity +
					                    simulationObjs [indexB].AngularVelocity.Cross (rb);

					Vector3 relativeVelocity = velocityA - velocityB;

					Vector3 tangentialVelocity = relativeVelocity -
					                             (collisionNormal.Dot (relativeVelocity)) * collisionNormal;

					#region Normal direction contact

					double error = collisionPointStr.IntersectionDistance * simulationParameters.BaumStabilization;
					double b = collisionNormal.Dot (relativeVelocity) * restitutionCoefficient -
						error;

					JacobianContact normalDirection = new JacobianContact (
						                                  indexA,
						                                  indexB,
						                                  null,
						                                  collisionNormal,
						                                  negCollisionNormal,
						                                  ra.Cross (collisionNormal),
						                                  rb.Cross (negCollisionNormal),
						                                  ConstraintType.Collision,
						                                  b,
						                                  0.0,
														  0.0,
						                                  0.0);

					#endregion

					#region Friction Contact

					JacobianContact[] frictionContact;

					if (Vector3.Length (tangentialVelocity) > 
						simulationParameters.ShiftToStaticFrictionTolerance) 
					{
						//Dynamic friction

						frictionContact = this.addDynamicFriction (
							indexA,
							indexB,
							simulationObjs,
							collisionPoint,
							collisionNormal,
							tangentialVelocity,
							ra,
							rb);
					} 
					else 
					{
						//Static friction

						frictionContact = this.addStaticFriction (
							indexA,
							indexB,
							simulationObjs,
							collisionPoint,
							collisionNormal,
							tangentialVelocity,
							ra,
							rb);
					}

					#endregion

					contactConstraints.Add (normalDirection);
					contactConstraints.Add (frictionContact[0]);
					contactConstraints.Add (frictionContact[1]);
				}
			}
			return contactConstraints;
		}

		private List<JacobianContact> BuildFixedJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> fixedConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.DistanceFromA;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.DistanceFromB;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix ();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix ();

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 angularError = this.getFixedAngularError (
															simulationObjectA,
															simulationObjectB,
															simulationJoint);

			#endregion

			#region Jacobian Constraint

			double constraintLimit = simulationJoint.K * linearError.x;

			//DOF 1

			fixedConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (-skewR1.r1c1, -skewR1.r1c2, -skewR1.r1c3),
				new Vector3 (skewR2.r1c1, skewR2.r1c2, skewR2.r1c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = simulationJoint.K * linearError.y;
			
			fixedConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (-skewR1.r2c1, -skewR1.r2c2, -skewR1.r2c3),
				new Vector3 (skewR2.r2c1, skewR2.r2c2, skewR2.r2c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));
			
			//DOF 3

			constraintLimit = simulationJoint.K * linearError.z;

			fixedConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (-skewR1.r3c1, -skewR1.r3c2, -skewR1.r3c3),
				new Vector3 (skewR2.r3c1, skewR2.r3c2, skewR2.r3c3),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = simulationJoint.K * 2.0 * angularError.x;

			fixedConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 5

			constraintLimit = simulationJoint.K * 2.0 * angularError.y;

			fixedConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 6

			constraintLimit = simulationJoint.K * 2.0 * angularError.z;
			
			fixedConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			#endregion

			return fixedConstraints;
		}
			
		private List<JacobianContact> BuildSliderJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> sliderConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 t1 = simulationObjectA.RotationMatrix * simulationJoint.Axis1;
			Vector3 t2 = simulationObjectA.RotationMatrix * simulationJoint.Axis2;

			Vector3 r1 =   simulationJoint.AnchorPoint - simulationObjectA.Position;
			Vector3 r2 =   simulationJoint.AnchorPoint - simulationObjectB.Position;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			Vector3 a = simulationObjectA.RotationMatrix * simulationJoint.Axis3;

			double sliderDistance = (r2 - r1).Dot (a);

			#endregion

			#region Init Angular

			Vector3 angularError = this.getFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				simulationJoint);

			#endregion

			#region Jacobian Constraint

			#region Constraints

			double constraintLimit = simulationJoint.K * 2.0 * angularError.x;

			//DOF 1

			sliderConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = simulationJoint.K * 2.0 * angularError.y;

			sliderConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = simulationJoint.K * 2.0 * angularError.z;

			sliderConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 1.0),
				new Vector3 (0.0, 0.0, -1.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = simulationJoint.K * Vector3.Dot (t1,linearError);

			sliderConstraints.Add (this.addDOF (
				indexA,
				indexB,
				t1,
				-1.0 * t1,
				Vector3.Cross (r1 + linearError, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 5

			constraintLimit = simulationJoint.K * Vector3.Dot (t2,linearError);

			sliderConstraints.Add (this.addDOF (
				indexA,
				indexB,
				t2,
				-1.0 * t2,
				Vector3.Cross (r1 + linearError, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			if (Math.Abs (sliderDistance) - simulationJoint.LinearLimitMin <= 0.0)
			{
				sliderConstraints.Add (this.addDOF (
					indexA,
					indexB,
					-1.0 * a,
					a,
					Vector3.Cross (r1, a),
					-1.0 * Vector3.Cross (r2, a),
					simulationObjectA,
					simulationObjectB,
					simulationJoint.K * (sliderDistance - simulationJoint.LinearLimitMin),
					simulationJoint.K * (sliderDistance - simulationJoint.LinearLimitMin),
					ConstraintType.Joint));
			}
				
			if (simulationJoint.LinearLimitMax - Math.Abs (sliderDistance) <= 0.0)
			{
				sliderConstraints.Add (this.addDOF (
					indexA,
					indexB,
					a,
					-1.0 * a,
					-1.0 * Vector3.Cross (r1, a),
					Vector3.Cross (r2, a),
					simulationObjectA,
					simulationObjectB,
					simulationJoint.K * (simulationJoint.LinearLimitMax - sliderDistance),
					simulationJoint.K * (simulationJoint.LinearLimitMax - sliderDistance),
					ConstraintType.Joint));
			}

			#endregion
			
			#endregion

			return sliderConstraints;
		}

		//TODO da verificare
		private List<JacobianContact> BuildHingeJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> sliderConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 t1 = simulationJoint.Axis1;
			Vector3 t2 = simulationJoint.Axis2;

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.DistanceFromA;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.DistanceFromB;

			Matrix3x3 skewP1 = Matrix3x3.GetSkewSymmetricMatrix (r1);
			Matrix3x3 skewP2 = Matrix3x3.GetSkewSymmetricMatrix (r2);

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 eulerA = Quaternion.GetEuler (simulationObjectA.RotationStatus);
			Vector3 eulerB = Quaternion.GetEuler (simulationObjectB.RotationStatus);

			Vector3 angularError = eulerB - eulerA;

			#endregion

			#region Jacobian Constraint

			double constraintLimit = simulationJoint.K * linearError.x;

			//DOF 1

			sliderConstraints.Add (this.addDOF (
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

			constraintLimit = simulationJoint.K * linearError.y;

			sliderConstraints.Add (this.addDOF (
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

			constraintLimit = simulationJoint.K * linearError.z;

			sliderConstraints.Add (this.addDOF (
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

			constraintLimit = simulationJoint.K * linearError.z;

			sliderConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3(0.0,0.0,0.0),
				new Vector3(0.0,0.0,0.0),
				t1,
				-1.0 * t1,
				simulationObjectA,
				simulationObjectB,
				simulationJoint.K * Vector3.Dot (t1,Vector3.ToZero ()),
				0,
				ConstraintType.Joint));

			//DOF 5

			sliderConstraints.Add (this.addDOF (
				indexA,
				indexB,
				new Vector3(0.0,0.0,0.0),
				new Vector3(0.0,0.0,0.0),
				t2,
				-1.0 * t2,
				simulationObjectA,
				simulationObjectB,
				simulationJoint.K * Vector3.Dot (t2,Vector3.ToZero ()),
				0,
				ConstraintType.Joint));

			#endregion

			return sliderConstraints;
		}



//		public List<JacobianContact> BuildJointsMatrix(
//			List<SimulationJoint> simulationJointList,
//			SimulationObject[] simulationObj)
//		{
//			List<JacobianContact> contactConstraints = new List<JacobianContact> ();
//
//			foreach (SimulationJoint simulationJoint in simulationJointList) 
//			{
//				int indexA = simulationJoint.IndexA;
//				int indexB = simulationJoint.IndexB;
//
//				foreach (Joint joint in simulationJoint.JointList) 
//				{
//					SimulationObject simulationObjectA = simulationObj [indexA];
//					SimulationObject simulationObjectB = simulationObj [indexB];
//
//					Vector3 ra = joint.Position - simulationObjectA.Position;
//					Vector3 rb = joint.Position - simulationObjectB.Position;
//
//					Vector3 velObjA = simulationObjectA.LinearVelocity +
//						Vector3.Cross (simulationObjectA.AngularVelocity, ra);
//
//					Vector3 velObjB = simulationObjectB.LinearVelocity +
//						Vector3.Cross (simulationObjectB.AngularVelocity, rb);
//
//					Vector3 relativeVelocity = velObjA - velObjB;
//
//					Vector3 r1 = simulationObjectA.RotationMatrix *
//						joint.DistanceFromA;
//
//					Vector3 r2 = simulationObjectB.RotationMatrix *
//						joint.DistanceFromB;
//
//					Vector3 p1 = simulationObjectA.Position + r1;
//					Vector3 p2 = simulationObjectB.Position + r2;
//
//					Vector3 dp = (p2 - p1);
//
//					JacobianContact Joint1 = this.setJointConstraint (
//						                         indexA,
//						                         indexB,
//						                         joint,
//						                         dp,
//						                         relativeVelocity,
//						                         new Vector3 (1.0, 0.0, 0.0),
//						                         ra,
//						                         rb);
//
//					JacobianContact Joint2 = this.setJointConstraint (
//						indexA,
//						indexB,                
//						joint,
//						dp,
//						relativeVelocity,
//						new Vector3 (0.0, 1.0, 0.0),
//						ra,
//						rb);
//
//					JacobianContact Joint3 = this.setJointConstraint (
//						indexA,
//						indexB,                 
//						joint,
//						dp,
//						relativeVelocity,
//						new Vector3 (0.0, 0.0, 1.0),
//						ra,
//						rb);
//
//					//Critical section
//					contactConstraints.Add (Joint1);
//					contactConstraints.Add (Joint2);
//					contactConstraints.Add (Joint3);
//				}
//			}
//			return contactConstraints;
//		}
			
		#endregion

		#region Common Methods

		/// <summary>
		/// Adds the friction.
		/// </summary>
		/// <returns>The friction.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="collisionPoint">Collision point.</param>
		/// <param name="normal">Normal.</param>
		/// <param name="tangentialVelocity">Tangential velocity.</param>
		/// <param name="frictionType">Friction type.</param>

		private JacobianContact[] addStaticFriction(
			int indexA,
			int indexB,
			SimulationObject[] simulationObjects,
			Vector3 collisionPoint,
			Vector3 normal,
			Vector3 tangentialVelocity,
			Vector3 ra,
			Vector3 rb)
		{
			JacobianContact[] friction = new JacobianContact[2];

			Vector3[] linearComponentA = new Vector3[2];
			Vector3[] linearComponentB = new Vector3[2];
			Vector3[] angularComponentA = new Vector3[2];
			Vector3[] angularComponentB = new Vector3[2];

			double constraintLimit = 0.5 * (simulationObjects [indexA].StaticFrictionCoeff +
				simulationObjects [indexB].StaticFrictionCoeff);

			linearComponentA [0] = GeometryUtilities.ProjectVectorOnPlane (normal);
			linearComponentB [0] = -1.0 * linearComponentA [0];

			angularComponentA [0] = ra.Cross (linearComponentA [0]);
			angularComponentB [0] = rb.Cross (linearComponentB [0]);

			linearComponentA [1] = linearComponentA [0].Cross (normal).Normalize ();
			linearComponentB [1] = -1.0 * linearComponentA [1];

			angularComponentA [1] = ra.Cross (linearComponentA [1]);
			angularComponentB [1] = rb.Cross (linearComponentB [1]);

			#region Jacobian Component

			friction [0] = new JacobianContact (
				indexA,
				indexB,
				-1,
				linearComponentA[0],
				linearComponentB[0],
				angularComponentA[0],
				angularComponentB[0],
				ConstraintType.Friction,
				0.0,
				constraintLimit,
				0.0,
				0.0);

			friction [1] = new JacobianContact (
				indexA,
				indexB,
				-2,
				linearComponentA[1],
				linearComponentB[1],
				angularComponentA[1],
				angularComponentB[1],
				ConstraintType.Friction,
				0.0,
				constraintLimit,
				0.0,
				0.0);

			#endregion

			return friction;

		}

		private JacobianContact[] addDynamicFriction(
			int indexA,
			int indexB,
			SimulationObject[] simulationObjects,
			Vector3 collisionPoint,
			Vector3 normal,
			Vector3 tangentialVelocity,
			Vector3 ra,
			Vector3 rb)
		{
			JacobianContact[] friction = new JacobianContact[2];

			Vector3[] linearComponentA = new Vector3[2];
			Vector3[] linearComponentB = new Vector3[2];
			Vector3[] angularComponentA = new Vector3[2];
			Vector3[] angularComponentB = new Vector3[2];

			double constraintLimit = 0.5 * (simulationObjects [indexA].DynamicFrictionCoeff +
				simulationObjects [indexB].DynamicFrictionCoeff);

			linearComponentA [0] = tangentialVelocity.Normalize ();
			linearComponentB [0] = -1.0 * linearComponentA [0];

			angularComponentA [0] = ra.Cross (linearComponentA [0]);
			angularComponentB [0] = rb.Cross (linearComponentB [0]);

			double B1 = linearComponentA [0].Dot (tangentialVelocity);

			linearComponentA [1] = tangentialVelocity.Cross (normal).Normalize ();
			linearComponentB [1] = -1.0 * linearComponentA [1];

			angularComponentA [1] = ra.Cross (linearComponentA [1]);
			angularComponentB [1] = rb.Cross (linearComponentB [1]);

			double B2 = linearComponentA [1].Dot (tangentialVelocity);

			#region Jacobian Component

			friction [0] = new JacobianContact (
				indexA,
				indexB,
				-1,
				linearComponentA[0],
				linearComponentB[0],
				angularComponentA[0],
				angularComponentB[0],
				ConstraintType.Friction,
				B1,
				constraintLimit,
				0.0,
				0.0);

			friction [1] = new JacobianContact (
				indexA,
				indexB,
				-2,
				linearComponentA[1],
				linearComponentB[1],
				angularComponentA[1],
				angularComponentB[1],
				ConstraintType.Friction,
				B2,
				constraintLimit,
				0.0,
				0.0);

			#endregion

			return friction;
		}

		private Vector3 getFixedAngularError(
			SimulationObject objectA,
			SimulationObject objectB,
			Joint simulationJoint)
		{
			Quaternion currentRelativeOrientation = objectA.RotationStatus.Inverse () *
				objectB.RotationStatus;

			Quaternion relativeOrientationError = simulationJoint.RelativeOrientation.Inverse () *
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

		private JacobianContact addDOF(
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
			ConstraintType type)
		{
			double jacobianVelocityValue = linearComponentA.Dot (simulationObjectA.LinearVelocity) +
			                               linearComponentB.Dot (simulationObjectB.LinearVelocity) +
			                               angularComponentA.Dot (simulationObjectA.AngularVelocity) +
			                               angularComponentB.Dot (simulationObjectB.AngularVelocity);

			double B = 0;

			if (!constraintLimitMin.HasValue &&
			    !constraintLimitMax.HasValue) 
			{
				B = jacobianVelocityValue;
			} 
			else if (constraintLimitMin.HasValue &&
			         !constraintLimitMax.HasValue) 
			{
				B = 0.0;

				double jacobianMass = this.getJacobianMassValue (
					                      linearComponentA,
					                      linearComponentB,
					                      angularComponentA,
					                      angularComponentB,
					                      simulationObjectA,
					                      simulationObjectB);

				constraintLimitMin = - (jacobianVelocityValue - constraintLimitMin) / jacobianMass;
				constraintLimitMax = double.MaxValue;
			} 
			else if (!constraintLimitMin.HasValue &&
			         constraintLimitMax.HasValue) 
			{

				B = jacobianVelocityValue - constraintLimitMax.Value;

				double jacobianMass = this.getJacobianMassValue (
					linearComponentA,
					linearComponentB,
					angularComponentA,
					angularComponentB,
					simulationObjectA,
					simulationObjectB);

				constraintLimitMin = double.MinValue;
				constraintLimitMax = -B / jacobianMass;;
			}
			else
			{
				if (constraintLimitMin.Value > constraintLimitMax.Value)
					throw new Exception ("Invalid constraint limit.");
				else if (constraintLimitMin == constraintLimitMax) 
				{
					B = jacobianVelocityValue - constraintLimitMin.Value;
					constraintLimitMin = double.MinValue;
					constraintLimitMax = double.MaxValue;
				}
			}

			return new JacobianContact (
				indexA,
				indexB,
				null,
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

		private double getJacobianMassValue(
			Vector3 linearComponentA,
			Vector3 linearComponentB,
			Vector3 angularComponentA,
			Vector3 angularComponentB,
			SimulationObject simulationObjectA,
			SimulationObject simulationObjectB)
		{
			double linearA = linearComponentA.Dot (
				                 linearComponentA * simulationObjectA.InverseMass);
			double linearB = linearComponentB.Dot (
				                 linearComponentB * simulationObjectB.InverseMass);

			double angularA = angularComponentA.Dot (
				                  simulationObjectA.InertiaTensor * angularComponentA);
			double angularB = angularComponentB.Dot (
				                  simulationObjectB.InertiaTensor * angularComponentB);

			return linearA + linearB + angularA + angularB;
		}


		#endregion

		#region Da eliminare
		/// <summary>
		/// Sets the joint constraint.
		/// </summary>
		/// <returns>The joint constraint.</returns>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="distanceParameter">Distance parameter.</param>
		/// <param name="relativeVelocity">Relative velocity.</param>
		/// <param name="axis">Axis.</param>
		private JacobianContact setJointConstraint(
			int indexA,
			int indexB,
			Joint joint,
			Vector3 distanceParameter,
			Vector3 relativeVelocity,
			Vector3 axis,
			Vector3 ra,
			Vector3 rb)
		{
			Vector3 linearComponentA = axis;

			Vector3 linearComponentB = -1.0 * linearComponentA;

			Vector3 angularComponentA = Vector3.Cross (ra, linearComponentA);

			Vector3 angularComponentB = Vector3.Cross (rb, linearComponentB);


			double error = joint.K * Vector3.Dot (axis, distanceParameter);
			double B = Vector3.Dot (axis, relativeVelocity) -
				joint.C * Vector3.Dot (axis, relativeVelocity) -
				error;

			return new JacobianContact (
				indexA,
				indexB,
				-1,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				ConstraintType.Joint,
				B,
				0.0,
				0.0,
				0.0);
		}

		#endregion

		#endregion
	}
}
