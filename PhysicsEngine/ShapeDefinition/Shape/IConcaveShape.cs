using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal interface IConcaveShape
    {
        AABB AABBox { get; }
        IGeometry[] ConvexShapesGeometry { get; }
        Vector3[] InputVertexPosition { get; }
        IGeometry ObjectGeometry { get; }
    }
}