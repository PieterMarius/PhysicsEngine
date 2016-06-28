using System.Collections.Generic;
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

