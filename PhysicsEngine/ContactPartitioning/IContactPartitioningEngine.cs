using System.Collections.Generic;
using ShapeDefinition;
using CollisionEngine;

namespace MonoPhysicsEngine
{
	public interface IContactPartitioningEngine
	{
		List<SpatialPartition> CalculateSpatialPartitioning (
			CollisionPointStructure[] collisionPoints,
			List<IConstraint> simulationJoint,
			IShape[] simulationObjects);

	}
}

