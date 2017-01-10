using System.Collections.Generic;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	internal interface IConstraintBuilder
	{
		List<JacobianContact> BuildJacobian(
			IShape[] simulationObjs,
			double? baumStabilization = null);
	}
}

