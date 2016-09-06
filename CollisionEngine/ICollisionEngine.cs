﻿using System.Collections.Generic;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public interface ICollisionEngine
	{
		List<CollisionPointStructure> Execute (
			ObjectGeometry[] objects,
			double minDistance);

		CollisionPointStructure GetIntersectionDistance(
			ObjectGeometry objectA,
			ObjectGeometry objectB);
	}
}

