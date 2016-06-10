using System;
using System.Collections.Generic;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	internal interface IConstraintBuilder
	{
		List<JacobianContact> BuildJacobian(int indexA,
											 int indexB,
											 SimulationObject[] simulationObjs);

	}
}

