using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public interface IShapeConvexDecomposition
    {
        List<ShapeDecompositionOutput> GetConvexShapeList(Vertex3Index[] vertexPosition, double precisionSize);
        List<ShapeDecompositionOutput> GetIntersectedShape(AABB box, AABB region, Vertex3Index[] vertexPosition, double precisionSize);
    }
}