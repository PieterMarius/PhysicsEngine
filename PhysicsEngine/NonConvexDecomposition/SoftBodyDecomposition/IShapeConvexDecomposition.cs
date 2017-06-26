using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public interface IShapeConvexDecomposition
    {
        List<List<Vertex3Index>> GetConvexShapeList(Vertex3Index[] vertexPosition, double precisionSize);
        List<List<Vertex3Index>> GetIntersectedShape(AABB box, Vertex3Index[] vertexPosition, double precisionSize);
    }
}