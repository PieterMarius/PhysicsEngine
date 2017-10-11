using SharpEngineMathUtility;
using System.Collections.Generic;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public sealed class VertexProperties
    {
        #region Fields

        public Vector3 Vertex { get; private set; }
        public List<int> Adjacency { get; private set; }
        public int? ID { get; private set; }
        public int?[] LinkedID { get; private set; }

        #endregion

        #region Constructor

        public VertexProperties(
            Vector3 vertex,
            List<int> adjacency,
            int? id,
            int?[] linkedID)
        {
            Vertex = vertex;
            Adjacency = adjacency;
            ID = id;
            LinkedID = linkedID;
        }

        public VertexProperties(
            Vector3 vertex,
            List<int> adjacency)
            : this(vertex, adjacency, null, null)
        { }

        public VertexProperties(
            Vector3 vertex)
            : this(vertex, null, null, null)
        { }

        public VertexProperties(
            Vector3 vertex,
            int? id)
            : this(vertex, null, id, null)
        { }

        public VertexProperties(
            Vector3 vertex,
            int? id,
            int?[] linkedID)
            : this(vertex, null, id, linkedID)
        { }

        public VertexProperties(
            Vector3 vertex,
            int?[] linkedID)
            : this(vertex, null, null, linkedID)
        { }

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
