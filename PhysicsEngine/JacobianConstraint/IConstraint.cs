using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public interface IConstraint
	{
		List<JacobianContact> BuildJacobian (int indexA,
		                                     int indexB,
		                                     SimulationObject[] simulationObjs);

		Vector3 GetStartAnchorPosition ();
		Vector3 GetAnchorPosition ();
	}
}

