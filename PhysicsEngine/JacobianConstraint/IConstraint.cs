using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public interface IConstraint
	{
		#region Get Methods

		Vector3 GetStartAnchorPosition ();
		Vector3 GetAnchorPosition ();
		int GetObjectIndexA();
		int GetObjectIndexB();

		#endregion

		#region Set Methods

		void SetAxis1Motor(double speedValue, double forceLimit);
		void SetAxis2Motor(double speedValue, double forceLimit);
		void AddTorque(double torqueAxis1, double torqueAxis2);

		#endregion
	}
}

