using System;
using System.Collections.Generic;
using CollisionEngine;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public interface IJacobianConstraintBuilder
	{
		List<JacobianContact> GetJacobianConstraint (
			List<CollisionPointStructure> collisionPointsStruct,
			List<ObjectConstraint> simulationJointList,
			SimulationObject[] simulationObjs,
			SimulationParameters simulationParameters);

	}
}

