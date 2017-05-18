﻿using System.Collections.Generic;
using ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	public interface ICollisionEngine
	{
        List<CollisionPointStructure> Execute(IShape[] objects);

        void SetCollisionDistance(double collisionDistance);
	}
}

