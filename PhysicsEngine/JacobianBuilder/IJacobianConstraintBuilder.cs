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
			List<SimulationJoint> simulationJointList,
			SimulationObject[] simulationObjs,
			SimulationParameters simulationParameters);

//		List<JacobianContact> BuildJointsMatrix (
//			List<SimulationJoint> simulationJointList,
//			SimulationObject[] simulationObj);
	}
}

