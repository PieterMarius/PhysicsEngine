using PhysicsEngineMathUtility;
using System.Collections.Generic;

namespace SimulationObjectDefinition
{
	public class ObjectGeometry
	{
		#region Object Properties

		public VertexAdjacency[] VertexPosition { get; private set; }
		public int[][] Triangle { get; private set; }
		public AABB AABBox { get; private set; }

		#endregion

		#region Constructor

		public ObjectGeometry (
			Vector3[] inputVertexPosition,
			int[][] inputTriangle)
		{
			Triangle = new int[inputTriangle.Length][];

            for (int i = 0; i < inputTriangle.Length; i++)
            {
                Triangle[i] = new int[3];
                Triangle[i][0] = inputTriangle[i][0];
                Triangle[i][1] = inputTriangle[i][1];
                Triangle[i][2] = inputTriangle[i][2];
            }

            SetVertexAdjacency(inputVertexPosition);
		}

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

		public void SetAABB(AABB box)
		{
			AABBox = box;
		}

        #endregion

        #region Private Methods

        private void SetVertexAdjacency(
            Vector3[] inputVertexPosition)
        {
            VertexPosition = new VertexAdjacency[inputVertexPosition.Length];

            for (int i = 0; i < VertexPosition.Length; i++)
            {
                VertexPosition[i] = new VertexAdjacency(inputVertexPosition[i], null);
                List<int> adjacencyList = new List<int>();

                for (int j = 0; j < Triangle.Length; j++)
                {
                    if (Triangle[j][0] == i)
                    {
                        AddAdjacencyItem(ref adjacencyList, Triangle[j][1]);
                        AddAdjacencyItem(ref adjacencyList, Triangle[j][2]);
                        continue;
                    }
                    if (Triangle[j][1] == i)
                    {
                        AddAdjacencyItem(ref adjacencyList, Triangle[j][0]);
                        AddAdjacencyItem(ref adjacencyList, Triangle[j][2]);
                        continue;
                    }
                    if (Triangle[j][2] == i)
                    {
                        AddAdjacencyItem(ref adjacencyList, Triangle[j][0]);
                        AddAdjacencyItem(ref adjacencyList, Triangle[j][1]);
                        continue;
                    }
                }
                VertexPosition[i].SetAdjacencyList(adjacencyList);
            }
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

