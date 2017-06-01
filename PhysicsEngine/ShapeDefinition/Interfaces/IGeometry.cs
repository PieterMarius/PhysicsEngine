using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public interface IGeometry
    {
        AABB AABBox { get; }
        ObjectGeometryType GeometryType { get; }
        IShape Shape { get; }
        TriangleIndexes[] Triangle { get; }
        VertexAdjacency[] VertexPosition { get; }
        Vector3[] RelativePosition { get; }

        void SetAABB(AABB box);
        void SetShape(IShape shape);
        void SetVertexPosition(Vector3 v, int index);
        void SetVertexPositions(Vector3[] v);
        void SetRelativePosition(Vector3[] relativePosition);
    }
}