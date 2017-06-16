using SharpEngineMathUtility;
using System.Collections.Generic;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public sealed class VertexAdjacency
    {
        #region Fields

        public Vector3 Vertex { get; private set; }
        public List<int> Adjacency { get; private set; }

        #endregion

        #region Constructor

        public VertexAdjacency(
            Vector3 vertex,
            List<int> adjacency)
        {
            Vertex = vertex;
            Adjacency = adjacency;
        }

        #endregion

        #region Public Methods

        public void SetVertexPosition(Vector3 position)
        {
            Vertex = position;
        }

        public void SetAdjacencyList(List<int> adjacencyList)
        {
            Adjacency = adjacencyList;
        }

        #endregion
    }
}
