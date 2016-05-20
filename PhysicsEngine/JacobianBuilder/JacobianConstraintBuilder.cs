using System;
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
				ContactConstraint.BuildJoints (
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
							FixedJointConstraint.BuildJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Slider:
						constraint.AddRange (
							SliderConstraint.BuildJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.BallAndSocket:
						constraint.AddRange (
							BallAndSocketConstraint.BuildJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Piston:
						constraint.AddRange (
							PistonConstraint.BuildJoint (
								simJoint.IndexA,
								simJoint.IndexB,
								joint,
								simulationObjs));
						break;

					case JointType.Hinge:
						constraint.AddRange (
							HingeConstraint.BuildJoint (
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
							Hinge2Constraint.BuildJoint (
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


		#endregion


		#endregion
	}
}

