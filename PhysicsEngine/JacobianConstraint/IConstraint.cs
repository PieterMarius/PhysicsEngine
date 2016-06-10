using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public interface IConstraint
	{
		Vector3 GetStartAnchorPosition ();
		Vector3 GetAnchorPosition ();
	}
}

