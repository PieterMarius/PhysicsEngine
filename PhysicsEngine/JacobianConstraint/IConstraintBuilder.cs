using System.Collections.Generic;
using ShapeDefinition;

namespace MonoPhysicsEngine
{
	internal interface IConstraintBuilder
	{
		List<JacobianContact> BuildJacobian(
			IShape[] simulationObjs,
			double? baumStabilization = null);
	}
}

