using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.ContinuosCollisionDetection
{
    internal interface ICCDEngine
    {
        double GetTimeOfImpact(IShape shapeA, IShape shapeB);
    }
}