using System.Collections.Generic;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public interface ICollisionEngine
	{
		List<CollisionPointStructure> Execute (
			SimulationObject[] objects,
			double minDistance);
	}
}

