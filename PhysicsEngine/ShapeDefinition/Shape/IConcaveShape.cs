using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal interface IConcaveShape
    {
        IGeometry[] ConvexShapesGeometry { get; }
        Vector3[] InputVertexPosition { get; }
        IGeometry ObjectGeometry { get; }
    }
}