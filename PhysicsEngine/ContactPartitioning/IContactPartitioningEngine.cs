using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.CollisionEngine;

namespace SharpPhysicsEngine
{
	internal interface IContactPartitioningEngine
	{
		List<SpatialPartition> CalculateSpatialPartitioning (
			CollisionPointStructure[] collisionPoints,
			List<IConstraint> simulationJoint,
			IShape[] simulationObjects);

	}
}

