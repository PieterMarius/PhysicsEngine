using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using CollisionEngine;

namespace MonoPhysicsEngine
{
	public interface IContactPartitioningEngine
	{
		List<SpatialPartition> calculateSpatialPartitioning (
			List<CollisionPointStructure> collisionPoints,
			List<SimulationJoint> simulationJoint,
			SimulationObject[] simulationObjects);

	}
}

