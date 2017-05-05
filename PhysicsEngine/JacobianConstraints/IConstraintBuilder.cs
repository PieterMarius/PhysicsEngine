using System.Collections.Generic;
using ShapeDefinition;

namespace SharpPhysicsEngine
{
	internal interface IConstraintBuilder
	{
        List<JacobianConstraint> BuildJacobian(double? baumStabilization = null);
	}
}

