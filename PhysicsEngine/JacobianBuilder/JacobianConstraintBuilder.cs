using System;
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

					Vector3 linearComponentA = (collisionPointStr.CollisionPoints [k].collisionNormal * -1.0).Normalize ();
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

					double correctionParameter = collisionPointStr.IntersectionDistance * simulationParameters.BaumStabilization;
					double b = linearComponentA.Dot (relativeVelocity) * restitutionCoefficient + correctionParameter;

					JacobianContact normalContact = this.addDOF (
														indexA,
														indexB,
														linearComponentA,
														linearComponentB,
														angularComponentA,
														angularComponentB,
														simulationObjs [indexA],
														simulationObjs [indexB],
														b,
														b,
														ConstraintType.Collision);

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
							linearComponentA,
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
							linearComponentA,
							tangentialVelocity,
							ra,
							rb);
					}

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

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (simulationJoint.Axis1);
			Vector3 t2 = Vector3.Cross (simulationJoint.Axis1, t1);

			t1 = simulationObjectA.RotationMatrix * t1;
			t2 = simulationObjectA.RotationMatrix * t2;

			Vector3 r1 =   simulationJoint.AnchorPoint - simulationObjectA.Position;
			Vector3 r2 =   simulationJoint.AnchorPoint - simulationObjectB.Position;

			Vector3 r = simulationObjectB.Position - simulationObjectA.Position;

			Vector3 linearError =  r - (simulationObjectA.RotationMatrix * simulationJoint.StartErrorAxis1);
			
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
				Vector3.Cross (r1, t1),
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
			double linearLimitMin = simulationJoint.Axis1.Dot (simulationJoint.LinearLimitMin);
			double linearLimitMax = simulationJoint.Axis1.Dot (simulationJoint.LinearLimitMax);

			sliderConstraints.Add (
				addLinearLimit(
				indexA,
				indexB,
				simulationJoint,
				simulationObjectA,
				simulationObjectB,
				simulationJoint.Axis1,
				r1,
				r2,
				linearLimitMin,
				linearLimitMax));

			#endregion
			
			#endregion

			return sliderConstraints;
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

			ballSocketConstraints.Add (this.addDOF (
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

			ballSocketConstraints.Add (this.addDOF (
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

			ballSocketConstraints.Add (this.addDOF (
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

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (simulationJoint.Axis1);
			Vector3 t2 = Vector3.Cross (simulationJoint.Axis1, t1);

			t1 = simulationObjectA.RotationMatrix * t1;
			t2 = simulationObjectA.RotationMatrix * t2;

			Vector3 r1 =   simulationJoint.AnchorPoint - simulationObjectA.Position;
			Vector3 r2 =   simulationJoint.AnchorPoint - simulationObjectB.Position;

			Vector3 r = simulationObjectB.Position - simulationObjectA.Position;

			Vector3 linearError =  r - (simulationObjectA.RotationMatrix * simulationJoint.StartErrorAxis1);

			#endregion

			#region Jacobian Constraint

			#region Constraints

			//DOF 1

			pistonConstraints.AddRange(
				addAngularLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB, 
					t1,
					0.0,
					0.0));

			//DOF 2

			pistonConstraints.AddRange(
				addAngularLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB, 
					t2,
					0.0,
					0.0));

			//DOF 3

			double constraintLimit = simulationJoint.K * Vector3.Dot (t1,linearError);

			pistonConstraints.Add (
				this.addDOF (
					indexA,
					indexB,
					1.0 * t1,
					-1.0 * t1,
					1.0 * Vector3.Cross (r1, t1),
					-1.0 * Vector3.Cross (r2, t1),
					simulationObjectA,
					simulationObjectB,
					constraintLimit,
					constraintLimit,
					ConstraintType.Joint));

			//DOF 4

			constraintLimit = simulationJoint.K * Vector3.Dot (t2,linearError);

			pistonConstraints.Add (
				this.addDOF (
					indexA,
					indexB,
					1.0 * t2,
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
			double linearLimitMin = simulationJoint.Axis1.Dot (simulationJoint.LinearLimitMin);
			double linearLimitMax = simulationJoint.Axis1.Dot (simulationJoint.LinearLimitMax);

			pistonConstraints.Add (
				addLinearLimit(
					indexA,
					indexB,
					simulationJoint,
					simulationObjectA,
					simulationObjectB,
					simulationJoint.Axis1,
					r1,
					r2,
					linearLimitMin,
					linearLimitMax));

			// Limit extraction
			double angularLimitMin = simulationJoint.Axis1.Dot (simulationJoint.AngularLimitMin);
			double angularLimitMax = simulationJoint.Axis1.Dot (simulationJoint.AngularLimitMax);

			Vector3 axisRotated = simulationObjectA.RotationMatrix * simulationJoint.Axis1;

			pistonConstraints.AddRange(
				addAngularLimit (
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

			Vector3 t1 = GeometryUtilities.GetPerpendicularVector (simulationJoint.Axis1);
			Vector3 t2 = Vector3.Cross (simulationJoint.Axis1, t1);

			t1 = simulationObjectA.RotationMatrix * t1;
			t2 = simulationObjectA.RotationMatrix * t2;

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

			#region Jacobian Constraint

			//DOF 1

			double constraintLimit = simulationJoint.K * linearError.x;

			hingeConstraints.Add (this.addDOF (
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

			hingeConstraints.Add (this.addDOF (
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

			hingeConstraints.Add (this.addDOF (
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

			hingeConstraints.AddRange(
				addAngularLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB, 
					t1,
					0.0,
					0.0));

			//DOF 5

			hingeConstraints.AddRange(
				addAngularLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB, 
					t2,
					0.0,
					0.0));

			#region Limit Constraints 

			//Limit extraction
			double angularLimitMax = simulationJoint.Axis1.Dot (simulationJoint.AngularLimitMax);
			double angularLimitMin = simulationJoint.Axis1.Dot (simulationJoint.AngularLimitMin);

			Vector3 axisRotated = simulationObjectA.RotationMatrix * simulationJoint.Axis1;

			hingeConstraints.AddRange(
				addAngularLimit (
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

			genericConstraints.Add (
				addLinearLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB,
					new Vector3 (1.0, 0.0, 0.0),
					r1, 
					r2,
					linearLimitMin,
					linearLimitMax));

			//DOF 2

			linearLimitMin = simulationJoint.LinearLimitMin.y;
			linearLimitMax = simulationJoint.LinearLimitMax.y;

			genericConstraints.Add (
				addLinearLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB,
					new Vector3 (0.0, 1.0, 0.0),
					r1, 
					r2,
					linearLimitMin,
					linearLimitMax));

			//DOF 3

			linearLimitMin = simulationJoint.LinearLimitMin.z;
			linearLimitMax = simulationJoint.LinearLimitMax.z;

			genericConstraints.Add (
				addLinearLimit (
					indexA, 
					indexB, 
					simulationJoint, 
					simulationObjectA, 
					simulationObjectB,
					new Vector3 (0.0, 0.0, 1.0),
					r1, 
					r2,
					linearLimitMin,
					linearLimitMax));

			//DOF 4

			double angularLimitMin = simulationJoint.AngularLimitMin.x;
			double angularLimitMax = simulationJoint.AngularLimitMax.x;

			genericConstraints.AddRange (
				addAngularLimit (
					indexA,
					indexB,
					simulationJoint,
					simulationObjectA,
					simulationObjectB,
					new Vector3 (1.0, 0.0, 0.0),
					angularLimitMin,
					angularLimitMax));
			
			//DOF 5

			angularLimitMin = simulationJoint.AngularLimitMin.y;
			angularLimitMax = simulationJoint.AngularLimitMax.y;

			genericConstraints.AddRange (
				addAngularLimit (
					indexA,
					indexB,
					simulationJoint,
					simulationObjectA,
					simulationObjectB,
					new Vector3 (0.0, 1.0, 0.0),
					angularLimitMin,
					angularLimitMax));

			//DOF 6

			angularLimitMin = simulationJoint.AngularLimitMin.z;
			angularLimitMax = simulationJoint.AngularLimitMax.z;

			genericConstraints.AddRange (
				addAngularLimit (
					indexA,
					indexB,
					simulationJoint,
					simulationObjectA,
					simulationObjectB,
					new Vector3 (0.0, 0.0, 1.0),
					angularLimitMin,
					angularLimitMax));

			#endregion

			return genericConstraints;

		}
						
		#endregion

		#region Common Methods

		private JacobianContact[] addStaticFriction(
			int indexA,
			int indexB,
			SimulationObject[] simulationObjects,
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

			#region Tangential Direction 1

			double constraintLimit = 0.5 * (simulationObjects [indexA].DynamicFrictionCoeff +
			                         simulationObjects [indexB].DynamicFrictionCoeff);

			linearComponentA = tangentialVelocity.Normalize ();
			linearComponentB = -1.0 * linearComponentA;

			angularComponentA = ra.Cross (linearComponentA);
			angularComponentB = -1.0 * rb.Cross (linearComponentA);

			friction [0] = this.addDOF (
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

			linearComponentA = tangentialVelocity.Cross (normal).Normalize ();
			linearComponentB = -1.0 * linearComponentA;

			angularComponentA = ra.Cross (linearComponentA);
			angularComponentB = -1.0 * rb.Cross (linearComponentA);

			friction [1] = this.addDOF (
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

		private double getRotationAngle(
			SimulationObject objectA,
			SimulationObject objectB,
			Vector3 rotationAxis,
			Joint simulationJoint)
		{
			Quaternion currentRelativeOrientation = objectA.RotationStatus.Inverse () *
			                                        objectB.RotationStatus;

			Quaternion relativeOrientation = simulationJoint.RelativeOrientation.Inverse () *
			                                 currentRelativeOrientation;

			Vector3 quaternionVectorPart = new Vector3 (
				                               relativeOrientation.b,
				                               relativeOrientation.c,
				                               relativeOrientation.d);

			quaternionVectorPart = objectA.RotationMatrix * quaternionVectorPart;

			double angle;

			if (quaternionVectorPart.Dot (rotationAxis) >= 0.0) 
			{
				angle = 2.0 * Math.Atan2 (quaternionVectorPart.Length (), relativeOrientation.a);
			} 
			else 
			{
				angle = 2.0 * Math.Atan2 (quaternionVectorPart.Length (), -relativeOrientation.a);
			}

			return (angle > Math.PI) ? angle - 2.0 * Math.PI : angle;
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

		private JacobianContact addLinearLimit (
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
			sliderAxis = simulationObjectA.RotationMatrix * sliderAxis;

			double sliderDistance = Math.Abs((r2 - r1).Dot (sliderAxis));

			Console.WriteLine ("Slider distance: " + sliderDistance);

			if (linearLimitMin == linearLimitMax) 
			{
				Vector3 r = simulationObjectB.Position - simulationObjectA.Position;

				Vector3 linearError =  r - (simulationObjectA.RotationMatrix * simulationJoint.StartErrorAxis1);

				double linearLimit = simulationJoint.K *
				                     sliderAxis.Dot (linearError);

				return this.addDOF (
					indexA,
					indexB,
					sliderAxis,
					-1.0 * sliderAxis,
					Vector3.Cross (r1, sliderAxis),
					-1.0 * Vector3.Cross (r2, sliderAxis),
					simulationObjectA,
					simulationObjectB,
					linearLimit,
					linearLimit,
					ConstraintType.Joint);
			}
			else if (sliderDistance < linearLimitMin) 
			{
				
				double linearLimit = simulationJoint.K *
				                     (linearLimitMin - sliderDistance);

				return this.addDOF (
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

				return this.addDOF (
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

		private List<JacobianContact> addAngularLimit (
			int indexA, 
			int indexB, 
			Joint simulationJoint,  
			SimulationObject simulationObjectA, 
			SimulationObject simulationObjectB,
			Vector3 rotationAxis,
			double angularLimitMin,
			double angularLimitMax)
		{
			List<JacobianContact> genericAngular = new List<JacobianContact> ();

			double angle = this.getRotationAngle (
				               simulationObjectA, 
				               simulationObjectB, 
				               rotationAxis, 
				               simulationJoint);

			Vector3 angularError = this.getFixedAngularError (
				                       simulationObjectA,
				                       simulationObjectB,
				                       simulationJoint);

			Vector3 zeroVector = new Vector3 (0.0, 0.0, 0.0);

			if (angularLimitMax == angularLimitMin) 
			{
				double angularLimit = simulationJoint.K *
				                      2.0 * rotationAxis.Dot (angularError);

				genericAngular.Add (this.addDOF (
					indexA, 
					indexB, 
					zeroVector, 
					zeroVector, 
					rotationAxis, 
					-1.0 * rotationAxis, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.Joint));
			} 
			else if (angle > angularLimitMax) 
			{
				
				double angularLimit = simulationJoint.K *
				                      (angle - angularLimitMax);

				genericAngular.Add (this.addDOF (
					indexA, 
					indexB, 
					zeroVector, 
					zeroVector, 
					rotationAxis, 
					-1.0 * rotationAxis, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.JointLimit));
			}
			else if (angle < angularLimitMin) 
			{
				double angularLimit = simulationJoint.K *
				                      (angularLimitMin - angle);

				genericAngular.Add (this.addDOF (
					indexA, 
					indexB, 
					zeroVector, 
					zeroVector, 
					-1.0 * rotationAxis, 
					rotationAxis, 
					simulationObjectA, 
					simulationObjectB, 
					angularLimit, 
					angularLimit, 
					ConstraintType.JointLimit));
			}

			return genericAngular;
		}
			
		#endregion

		#endregion
	}
}

