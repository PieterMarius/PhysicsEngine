using ShapeDefinition;
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
		int GetKeyIndex();

		#endregion

		#region Set Methods

		void SetObjectIndexA(int index);
		void SetObjectIndexB(int index);
		void SetLinearLimit(double linearLimitMin, double linearLimitMax);
		void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax);
		void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax);
		void SetAxis1Motor(double speedValue, double forceLimit);
		void SetAxis2Motor(double speedValue, double forceLimit);
		void AddTorque(ConvexShape[] objects, double torqueAxis1, double torqueAxis2);
		void SetRestoreCoefficient(double restoreCoefficient);

		#endregion
	}
}

