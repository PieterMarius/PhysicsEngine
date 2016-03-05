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

		List<JacobianContact> BuildContactJoints (
			List<CollisionPointStructure> collisionPointsStruct,
			SimulationObject[] simulationObjs,
			SimulationParameters simulationParameters);

		List<JacobianContact> BuildFixedJoint (
			int indexA,
			int indexB,
			Joint simulationJoint,
			SimulationObject[] simulationObjs);

//		List<JacobianContact> BuildJointsMatrix (
//			List<SimulationJoint> simulationJointList,
//			SimulationObject[] simulationObj);
	}
}

