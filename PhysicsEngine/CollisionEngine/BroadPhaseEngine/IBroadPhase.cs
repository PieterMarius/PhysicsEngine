using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal interface IBroadPhase
    {
        List<CollisionPair> Execute(
            AABB[] boxs,
            double distanceTolerance);
    }
}
