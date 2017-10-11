using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public sealed class ShapeDecompositionOutput
    {
        public List<Vertex3Index> Vertex3Idx { get; private set; }
        public AABB Region { get; private set; }

        public ShapeDecompositionOutput(
            List<Vertex3Index> vertex3Index,
            AABB region)
        {
            Vertex3Idx = vertex3Index;
            Region = region;
        }

        public void AddVertex3Index(List<Vertex3Index> vertex3Index)
        {
            Vertex3Idx.AddRange(vertex3Index);
        }
    }
}
