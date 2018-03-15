using System.Collections.Generic;

namespace SharpEngineMathUtility
{
    public class Graph
    {
        public Graph() { }
        public Graph(int[] vertices, HashSetStruct[] edges)
        {
            foreach (var vertex in vertices)
                AddVertex(vertex);

            foreach (var edge in edges)
                AddEdge(edge);
        }

        public Graph(int verticesCount)
        {
            for (int i = 0; i < verticesCount; i++)
            {
                AddVertex(i);
            } 
        }

        public Dictionary<int, List<int>> AdjacencyList { get; } = new Dictionary<int, List<int>>();

        public void AddVertex(int vertex)
        {
            AdjacencyList[vertex] = new List<int>();
        }

        public void AddEdge(HashSetStruct edge)
        {
            if (AdjacencyList.TryGetValue(edge.ID_A, out List<int> listA) &&
                AdjacencyList.TryGetValue(edge.ID_B, out List<int> listB))
            {
                listA.Add(edge.ID_B);
                listB.Add(edge.ID_A);
            }
        }

        public void AddEdge(int keyValue, List<int> edgeValue)
        {
            AdjacencyList[keyValue].AddRange(edgeValue);
        }
    }
}
