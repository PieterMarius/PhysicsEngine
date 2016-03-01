using System;

namespace SimulationObjectDefinition
{
	public enum ConstraintType
	{
		Collision,
		StaticFriction,
		DynamicFriction,
		Joint, //TODO da eliminare 
		PointToPoint,
		Hinge,
		Slider,
		ConeTwist,
		Generic6DOF,
		Fixed,
	}
}

