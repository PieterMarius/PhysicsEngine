using System;
using System.Collections.Generic;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	internal interface IConstraintBuilder
	{
		List<JacobianContact> BuildJacobian(SimulationObject[] simulationObjs);

	}
}

