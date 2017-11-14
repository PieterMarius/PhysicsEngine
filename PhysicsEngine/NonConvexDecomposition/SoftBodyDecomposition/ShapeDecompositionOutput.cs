using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    internal sealed class ShapeDecompositionOutput
    {

        #region Fields

        public HashSet<Vertex3Index> Vertex3Idx { get; private set; }
        public AABB Region { get; private set; }

        #endregion

        #region Constructor

        public ShapeDecompositionOutput(
            HashSet<Vertex3Index> vertex3Index,
            AABB region)
        {
            Vertex3Idx = vertex3Index;
            Region = region;
        }

        #endregion

        #region Public Methods

        public void AddVertex3Index(HashSet<Vertex3Index> vertex3Index)
        {
            Vertex3Idx.UnionWith(vertex3Index);
        }

        #endregion
    }
}
