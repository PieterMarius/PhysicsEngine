using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using ObjectDefinition;

namespace CollisionEngine
{
	public interface ICollisionEngine
	{
		List<CollisionPointStructure> RunTestCollision (
			ObjectGeometry[] objects,
			double minDistance);

	}
}

