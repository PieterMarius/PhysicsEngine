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
			List<ObjectConstraint> simulationJointList,
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

			foreach(ObjectConstraint item in simulationJointList)
			{
				foreach(IConstraintBuilder constraintItem in item.ConstraintList)
				{
					constraint.AddRange (
						constraintItem.BuildJacobian (
							item.IndexA,
							item.IndexB,
							simulationObjs));
				}
			}

			#endregion

			return constraint;
		}

		#endregion

	}
}

