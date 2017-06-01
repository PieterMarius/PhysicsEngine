using SharpEngineMathUtility;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.ShapeDefinition
{
	public class Geometry : IGeometry
    {
		#region Object Properties

        /// <summary>
        /// Vertex Position
        /// </summary>
		public VertexAdjacency[] VertexPosition { get; private set; }
        
        /// <summary>
        /// Triangle Index
        /// </summary>
        public TriangleIndexes[] Triangle { get; private set; }
        
        /// <summary>
        /// Bounding Box
        /// </summary>
        public AABB AABBox { get; private set; }
        
        /// <summary>
        /// Get the geometry property of object
        /// </summary>
        public ObjectGeometryType GeometryType { get; private set; }
        /// <summary>
        /// 
        /// </summary>
        public Vector3[] RelativePosition { get; private set; }

        /// <summary>
        /// 
        /// </summary>
        public IShape Shape { get; private set; }

        #endregion

        #region Constructor

        public Geometry (
            IShape shape,
			Vector3[] inputVertexPosition,
            TriangleIndexes[] inputTriangle,
            ObjectGeometryType geometryType,
            bool getAdjacencyList)
		{
            Shape = shape;
            GeometryType = geometryType;

            if (inputTriangle != null)
            {
                Triangle = inputTriangle;
                
                SetVertexAdjacency(inputVertexPosition, getAdjacencyList);
                
            }
            else
            {
                SetVertexAdjacency(inputVertexPosition, getAdjacencyList);
            }
        }

        public Geometry(
            IShape shape,
            Vector3[] inputVertexPosition,
            ObjectGeometryType geometryType)
            : this(shape, inputVertexPosition, null, geometryType, false)
        { }

        #endregion

            #region Public Methods

        public void SetVertexPosition(Vector3 v, int index)
		{
			if (VertexPosition != null && 
				VertexPosition.Length > index ) 
			{
                VertexPosition[index].SetVertexPosition(v);
			}
		}

		public void SetVertexPositions(Vector3[] v)
		{
			for (int i = 0; i < v.Length; i++)
				VertexPosition [i].SetVertexPosition(v[i]);
		}

        public void SetRelativePosition(Vector3[] relativePosition)
        {
            RelativePosition = relativePosition;
        }

		public void SetAABB(AABB box)
		{
			AABBox = box;
		}

        public void SetShape(IShape shape)
        {
            Shape = shape;
        }

        #endregion

        #region Private Methods

        private void SetVertexAdjacency(
            Vector3[] inputVertexPosition,
            bool getAdjacencyList)
        {
            VertexPosition = new VertexAdjacency[inputVertexPosition.Length];

            Parallel.For(0,
                VertexPosition.Length,
                new ParallelOptions { MaxDegreeOfParallelism = 4 },
                i =>
                {
                    VertexPosition[i] = new VertexAdjacency(inputVertexPosition[i], null);

                    if (Triangle != null && 
                        getAdjacencyList)
                    {
                        List<int> adjacencyList = new List<int>();

                        for (int j = 0; j < Triangle.Length; j++)
                        {
                            if (Triangle[j].a == i)
                            {
                                AddAdjacencyItem(ref adjacencyList, Triangle[j].b);
                                AddAdjacencyItem(ref adjacencyList, Triangle[j].c);
                                continue;
                            }
                            if (Triangle[j].b == i)
                            {
                                AddAdjacencyItem(ref adjacencyList, Triangle[j].a);
                                AddAdjacencyItem(ref adjacencyList, Triangle[j].c);
                                continue;
                            }
                            if (Triangle[j].c == i)
                            {
                                AddAdjacencyItem(ref adjacencyList, Triangle[j].a);
                                AddAdjacencyItem(ref adjacencyList, Triangle[j].b);
                                continue;
                            }
                        }
                        VertexPosition[i].SetAdjacencyList(adjacencyList);
                    }
                });
        }

        private void AddAdjacencyItem(
            ref List<int> adjacencyList,
            int itemIndex)
        {
            if (!adjacencyList.Contains(itemIndex))
                adjacencyList.Add(itemIndex);
        }

        #endregion
    }

    public class VertexAdjacency
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

