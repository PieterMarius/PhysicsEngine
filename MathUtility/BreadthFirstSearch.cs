using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpEngineMathUtility
{
    public class BreadthFirstSearch
    {
        public HashSet<T> BFS<T>(Graph<T> graph, T start)
        {
            var visited = new HashSet<T>();
            
            if (!graph.AdjacencyList.ContainsKey(start))
                return visited;
                
            var queue = new Queue<T>();
            queue.Enqueue(start);

            while (queue.Count > 0) {
                var vertex = queue.Dequeue();

                if (visited.Contains(vertex))
                    continue;

                visited.Add(vertex);

                foreach (var neighbor in graph.AdjacencyList[vertex])
                {
                    if (!visited.Contains(neighbor))
                    {
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return visited;
        }

        public Dictionary<T, int> GetLevelBFS<T>(Graph<T> graph, T start)
        {
            var visited = new HashSet<T>();
            var level = new Dictionary<T, int>
            {
                { start, 0 }
            };

            if (!graph.AdjacencyList.ContainsKey(start))
                return level;

            var queue = new Queue<T>();
            queue.Enqueue(start);

            while (queue.Count > 0)
            {
                var vertex = queue.Dequeue();

                if (visited.Contains(vertex))
                    continue;

                visited.Add(vertex);

                foreach (var neighbor in graph.AdjacencyList[vertex])
                {
                    if (!visited.Contains(neighbor))
                    {
                        level.TryGetValue(vertex, out int value);
                        if (!level.ContainsKey(neighbor))
                            level.Add(neighbor, value + 1);
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return level;
        }
    }



    public class Graph<T>
    {
        public Graph() { }
        public Graph(IEnumerable<T> vertices, IEnumerable<Tuple<T, T>> edges)
        {
            foreach (var vertex in vertices)
                AddVertex(vertex);

            foreach (var edge in edges)
                AddEdge(edge);
        }

        public Dictionary<T, HashSet<T>> AdjacencyList { get; } = new Dictionary<T, HashSet<T>>();

        public void AddVertex(T vertex)
        {
            AdjacencyList[vertex] = new HashSet<T>();
        }

        public void AddEdge(Tuple<T, T> edge)
        {
            if (AdjacencyList.ContainsKey(edge.Item1) && AdjacencyList.ContainsKey(edge.Item2))
            {
                AdjacencyList[edge.Item1].Add(edge.Item2);
                AdjacencyList[edge.Item2].Add(edge.Item1);
            }
        }
    }
}
