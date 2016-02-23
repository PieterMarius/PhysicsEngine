using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public interface ICollisionEngine
	{
		List<CollisionPointStructure> RunTestCollision (
			ObjectGeometry[] objects,
			double minDistance);

	}
}

