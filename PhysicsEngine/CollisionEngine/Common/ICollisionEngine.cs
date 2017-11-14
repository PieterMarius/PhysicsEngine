using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal interface ICollisionEngine
	{
        List<CollisionPointStructure> Execute(IShape[] objects);

        void SetCollisionDistance(double collisionDistance);
	}
}

