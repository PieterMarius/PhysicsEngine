﻿using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using CollisionEngine;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public class JacobianConstraintBuilder: IJacobianConstraintBuilder
	{
		#region Private Fields

		private const Double tolerance = 1E-30;

		#endregion

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

					case JointType.BallAndSocket:
						constraint.AddRange (
							this.BuildBallSocketJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Piston:
						constraint.AddRange (
							this.BuildPistonJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Hinge:
						constraint.AddRange (
							this.BuildHingeJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Generic6DOF:
						constraint.AddRange (
							this.BuildGeneric6DOFJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Hinge2:
						constraint.AddRange (
							this.BuildHinge2Joint (
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

					JacobianContact normalContact = JacobianBuilderCommon.GetDOF (
														indexA,
														indexB,
														linearComponentA,
														linearComponentB,
														angularComponentA,
														angularComponentB,
														simulationObjs [indexA],
														simulationObjs [indexB],
														correctedBounce,
														correctedBounce,
														ConstraintType.Collision);

					#endregion

					#region Friction Contact

					JacobianContact[] frictionContact = this.addFriction (
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
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix ();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix ();

			#endregion

			#region Init Angular

			Vector3 angularError = JacobianBuilderCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				simulationJoint);

			#endregion

			#region Jacobian Constraint

			double constraintLimit = simulationJoint.K * linearError.x;

			//DOF 1

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF(
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
			
			fixedConstraints.Add (JacobianBuilderCommon.GetDOF(
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

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
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
			
			fixedConstraints.Add (JacobianBuilderCommon.GetDOF (
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
			
		private List<JacobianContact> BuildBallSocketJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> ballSocketConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Matrix3x3 skewR1 = r1.GetSkewSymmetricMatrix ();
			Matrix3x3 skewR2 = r2.GetSkewSymmetricMatrix ();

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Jacobian Constraint

			double constraintLimit = simulationJoint.K * linearError.x;

			//DOF 1

			ballSocketConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			ballSocketConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			ballSocketConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			#endregion

			return ballSocketConstraints;
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

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * simulationJoint.JointActDirection1;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3 t2 = Vector3.Cross (sliderAxis, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;
			
			#endregion

			#region Init Angular

			Vector3 angularError = JacobianBuilderCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				simulationJoint);

			#endregion

			#region Jacobian Constraint

			#region Constraints

			double constraintLimit = simulationJoint.K * 2.0 * angularError.x;

			//DOF 1

			sliderConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (-1.0, 0.0, 0.0),
				new Vector3 (1.0, 0.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 2

			constraintLimit = simulationJoint.K * 2.0 * angularError.y;

			sliderConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, -1.0, 0.0),
				new Vector3 (0.0, 1.0, 0.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 3

			constraintLimit = simulationJoint.K * 2.0 * angularError.z;

			sliderConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, 0.0),
				new Vector3 (0.0, 0.0, -1.0),
				new Vector3 (0.0, 0.0, 1.0),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = simulationJoint.K * Vector3.Dot (t1,linearError);

			sliderConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 5

			constraintLimit = simulationJoint.K * Vector3.Dot (t2,linearError);

			sliderConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				t2,
				-1.0 * t2,
				Vector3.Cross (r1, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 


			// Limit extraction
			double linearLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.LinearLimitMin);
			double linearLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.LinearLimitMax);

			sliderConstraints.Add (
				JacobianBuilderCommon.GetLinearLimit(
				indexA,
				indexB,
				simulationJoint,
				simulationObjectA,
				simulationObjectB,
				sliderAxis,
				r1,
				r2,
				linearLimitMin,
				linearLimitMax));

			#endregion
			
			#endregion

			return sliderConstraints;
		}
			
		private List<JacobianContact> BuildPistonJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> pistonConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 sliderAxis = simulationObjectA.RotationMatrix * simulationJoint.JointActDirection1;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (sliderAxis).Normalize ();
			Vector3 t2 = Vector3.Cross (sliderAxis, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			Vector3 angularError = sliderAxis.Cross (
				(simulationObjectB.RotationMatrix * simulationJoint.JointActDirection1));

			#region Jacobian Constraint

			#region Constraints

			//DOF 1

			double angularLimit = simulationJoint.K *
				t1.Dot (angularError);

			pistonConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					1.0 * t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			//DOF 2

			angularLimit = simulationJoint.K *
				t2.Dot (angularError);

			pistonConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					1.0 * t2, 
					-1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			//DOF 3

			double constraintLimit = simulationJoint.K * Vector3.Dot (t1,linearError);

			Console.WriteLine ("linear error:" + constraintLimit);

			pistonConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				t1,
				-1.0 * t1,
				Vector3.Cross (r1, t1),
				-1.0 * Vector3.Cross (r2, t1),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			//DOF 4

			constraintLimit = simulationJoint.K * Vector3.Dot (t2,linearError);

			Console.WriteLine ("linear error:" + constraintLimit);

			pistonConstraints.Add (JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				t2,
				-1.0 * t2,
				Vector3.Cross (r1, t2),
				-1.0 * Vector3.Cross (r2, t2),
				simulationObjectA,
				simulationObjectB,
				constraintLimit,
				constraintLimit,
				ConstraintType.Joint));

			#endregion

			#region Limit Constraints 

			// Limit extraction
			double linearLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.LinearLimitMin);
			double linearLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.LinearLimitMax);

			pistonConstraints.Add (
				JacobianBuilderCommon.GetLinearLimit(
					indexA,
					indexB,
					simulationJoint,
					simulationObjectA,
					simulationObjectB,
					sliderAxis,
					r1,
					r2,
					linearLimitMin,
					linearLimitMax));

			// Limit extraction
			double angularLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMin);
			double angularLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMax);

			pistonConstraints.Add(
				JacobianBuilderCommon.GetAngularLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB, 
					sliderAxis,
					angularLimitMin,
					angularLimitMax));
			
			#endregion

			#endregion

			return pistonConstraints;
		}
			
		private List<JacobianContact> BuildHingeJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> hingeConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 axisRotated = simulationObjectA.RotationMatrix * simulationJoint.JointActDirection1;

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (axisRotated).Normalize ();
			Vector3 t2 = Vector3.Cross (axisRotated, t1).Normalize ();

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Matrix3x3 skewP1 = Matrix3x3.GetSkewSymmetricMatrix (r1);
			Matrix3x3 skewP2 = Matrix3x3.GetSkewSymmetricMatrix (r2);

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 angularError = JacobianBuilderCommon.GetFixedAngularError (
				simulationObjectA,
				simulationObjectB,
				simulationJoint);

			#endregion


			#region Jacobian Constraint

			//DOF 1

			double constraintLimit = simulationJoint.K * linearError.x;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF(
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

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			double angularLimit = simulationJoint.K *
				t1.Dot (angularError);

			hingeConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t1, 
					1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			//DOF 5

			angularLimit = simulationJoint.K *
				t2.Dot (angularError);

			hingeConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					-1.0 * t2, 
					1.0 * t2, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			#region Limit Constraints 

			//Limit extraction
			double angularLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMax);
			double angularLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMin);

			hingeConstraints.Add(JacobianBuilderCommon.GetAngularLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB, 
					axisRotated,
					angularLimitMin,
					angularLimitMax));
			
			#endregion

			#endregion

			return hingeConstraints;
		}

		private List<JacobianContact> BuildHinge2Joint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> hingeConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			Matrix3x3 skewP1 = Matrix3x3.GetSkewSymmetricMatrix (r1);
			Matrix3x3 skewP2 = Matrix3x3.GetSkewSymmetricMatrix (r2);

			Vector3 p1 = simulationObjectA.Position + r1;
			Vector3 p2 = simulationObjectB.Position + r2;

			Vector3 linearError = p2 - p1;

			#endregion

			#region Init Angular

			Vector3 hingeAxis = simulationObjectA.RotationMatrix * simulationJoint.JointActDirection1;

			Vector3 rotationAxis = simulationObjectB.RotationMatrix * simulationJoint.JointActDirection2;

			double k = hingeAxis.Dot (rotationAxis);
			Vector3 tempPerpendicular = rotationAxis - k * hingeAxis;
			Vector3 t1 = hingeAxis.Cross (tempPerpendicular).Normalize ();

			#endregion

			#region Jacobian Constraint

			//DOF 1

			double constraintLimit = simulationJoint.K * linearError.x;

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF(
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

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			hingeConstraints.Add (JacobianBuilderCommon.GetDOF (
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

			double angularLimit = simulationJoint.K * (-k);

			hingeConstraints.Add (
				JacobianBuilderCommon.GetDOF (
					indexA, 
					indexB, 
					new Vector3(), 
					new Vector3(), 
					t1, 
					-1.0 * t1, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));

			#region Limit Constraints 

			//Limit extraction
			double angularLimitMax = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMax);
			double angularLimitMin = simulationJoint.JointActDirection1.Dot (simulationJoint.AngularLimitMin);

//			hingeConstraints.Add(JacobianBuilderCommon.GetAngularLimit (
//				indexA, 
//				indexB, 
//				simulationJoint, 
//				simulationObjectA, 
//				simulationObjectB, 
//				hingeAxis,
//				angularLimitMin,
//				angularLimitMax));

			#endregion

			#endregion

			return hingeConstraints;
		}

		private List<JacobianContact> BuildGeneric6DOFJoint(
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> genericConstraints = new List<JacobianContact> ();

			SimulationObject simulationObjectA = simulationObjs [indexA];
			SimulationObject simulationObjectB = simulationObjs [indexB];

			#region Init Linear

			Vector3 r1 = simulationObjectA.RotationMatrix *
				simulationJoint.StartErrorAxis1;

			Vector3 r2 = simulationObjectB.RotationMatrix *
				simulationJoint.StartErrorAxis2;

			#endregion

			#region Jacobian Constraint

			//DOF 1

			// Limit extraction
			double linearLimitMin = simulationJoint.LinearLimitMin.x;
			double linearLimitMax = simulationJoint.LinearLimitMax.x;

//			genericConstraints.Add (
//				addLinearLimit (
//					indexA, 
//					indexB, 
//					simulationJoint, 
//					simulationObjectA, 
//					simulationObjectB,
//					new Vector3 (1.0, 0.0, 0.0),
//					r1, 
//					r2,
//					linearLimitMin,
//					linearLimitMax));
//
//			//DOF 2
//
//			linearLimitMin = simulationJoint.LinearLimitMin.y;
//			linearLimitMax = simulationJoint.LinearLimitMax.y;
//
//			genericConstraints.Add (
//				addLinearLimit (
//					indexA, 
//					indexB, 
//					simulationJoint, 
//					simulationObjectA, 
//					simulationObjectB,
//					new Vector3 (0.0, 1.0, 0.0),
//					r1, 
//					r2,
//					linearLimitMin,
//					linearLimitMax));
//
//			//DOF 3
//
//			linearLimitMin = simulationJoint.LinearLimitMin.z;
//			linearLimitMax = simulationJoint.LinearLimitMax.z;
//
//			genericConstraints.Add (
//				addLinearLimit (
//					indexA, 
//					indexB, 
//					simulationJoint, 
//					simulationObjectA, 
//					simulationObjectB,
//					new Vector3 (0.0, 0.0, 1.0),
//					r1, 
//					r2,
//					linearLimitMin,
//					linearLimitMax));
//
//			//DOF 4
//
//			double angularLimitMin = simulationJoint.AngularLimitMin.x;
//			double angularLimitMax = simulationJoint.AngularLimitMax.x;
//
//			genericConstraints.Add (
//				addAngularLimit (
//					indexA,
//					indexB,
//					simulationJoint,
//					simulationObjectA,
//					simulationObjectB,
//					new Vector3 (1.0, 0.0, 0.0),
//					angularLimitMin,
//					angularLimitMax));
//			
//			//DOF 5
//
//			angularLimitMin = simulationJoint.AngularLimitMin.y;
//			angularLimitMax = simulationJoint.AngularLimitMax.y;
//
//			genericConstraints.Add (
//				addAngularLimit (
//					indexA,
//					indexB,
//					simulationJoint,
//					simulationObjectA,
//					simulationObjectB,
//					new Vector3 (0.0, 1.0, 0.0),
//					angularLimitMin,
//					angularLimitMax));
//
//			//DOF 6
//
//			angularLimitMin = simulationJoint.AngularLimitMin.z;
//			angularLimitMax = simulationJoint.AngularLimitMax.z;
//
//			genericConstraints.Add (
//				addAngularLimit (
//					indexA,
//					indexB,
//					simulationJoint,
//					simulationObjectA,
//					simulationObjectB,
//					new Vector3 (0.0, 0.0, 1.0),
//					angularLimitMin,
//					angularLimitMax));

			#endregion

			return genericConstraints;

		}
						
		#endregion

		#region Common Methods

		private JacobianContact[] addFriction(
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

			friction [0] = JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				simulationObjects [indexA],
				simulationObjects [indexB],
				constraintLimit,
				0.0,
				ConstraintType.Friction,
				-1);

			#endregion

			#region Tangential Direction 2

			linearComponentA = t.Cross (normal).Normalize ();
			linearComponentB = -1.0 * linearComponentA;

			angularComponentA = ra.Cross (linearComponentA);
			angularComponentB = -1.0 * rb.Cross (linearComponentA);

			friction [1] = JacobianBuilderCommon.GetDOF (
				indexA,
				indexB,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				simulationObjects [indexA],
				simulationObjects [indexB],
				constraintLimit,
				0.0,
				ConstraintType.Friction,
				-2);

			#endregion

			return friction;
		}

		#endregion


		#endregion
	}
}

