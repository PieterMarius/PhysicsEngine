using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal interface INarrowPhase
    {
        List<CollisionPointStructure> Execute(
            IShape[] shapes, 
            List<CollisionPair> collisionPairs, 
            HashSet<HashSetStruct> ignoreList);

        CollisionPointStructure Execute(
            IShape shapeA,
            IShape shapeB);
    }
}