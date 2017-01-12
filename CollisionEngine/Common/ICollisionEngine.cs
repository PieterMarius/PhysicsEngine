using System.Collections.Generic;
using ShapeDefinition;

namespace CollisionEngine
{
	public interface ICollisionEngine
	{
		List<CollisionPointStructure> Execute (
			IShape[] objects,
			double minDistance);
	}
}

