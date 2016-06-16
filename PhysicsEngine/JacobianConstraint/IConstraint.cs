using System;
using System.Collections.Generic;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public interface IConstraint
	{
		#region Get Methods

		Vector3 GetAnchorPosition ();
		JointType GetJointType();
		int GetObjectIndexA();
		int GetObjectIndexB();

		#endregion

		#region Set Methods

		void SetLinearLimit(double linearLimitMin, double linearLimitMax);
		void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax);
		void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax);
		void SetAxis1Motor(double speedValue, double forceLimit);
		void SetAxis2Motor(double speedValue, double forceLimit);
		void AddTorque(SimulationObject[] simObj, double torqueAxis1, double torqueAxis2);

		#endregion
	}
}

