using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
	internal interface IConstraintBuilder
	{
        List<JacobianConstraint> BuildJacobian(double? baumStabilization = null);
	}
}

