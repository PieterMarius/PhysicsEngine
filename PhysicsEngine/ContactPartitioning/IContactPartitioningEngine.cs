using System.Collections.Generic;
using ShapeDefinition;
using CollisionEngine;

namespace SharpPhysicsEngine
{
	public interface IContactPartitioningEngine
	{
		List<SpatialPartition> CalculateSpatialPartitioning (
			CollisionPointStructure[] collisionPoints,
			List<IConstraint> simulationJoint,
			IShape[] simulationObjects);

	}
}

