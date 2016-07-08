using System.Collections.Generic;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public interface ICollisionEngine
	{
		List<CollisionPointStructure> RunCollisionDetection (
			ObjectGeometry[] objects,
			double minDistance);
	}
}

