using System.Collections.Generic;
using ShapeDefinition;

namespace SharpPhysicsEngine
{
	internal interface IConstraintBuilder
	{
		List<JacobianConstraint> BuildJacobian(
			IShape[] simulationObjs,
			double? baumStabilization = null);
	}
}

