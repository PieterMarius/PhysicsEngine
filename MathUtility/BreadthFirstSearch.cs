using System;
using System.Collections.Generic;

namespace SharpEngineMathUtility
{
    public class BreadthFirstSearch
    {
        public HashSet<int> BFS(Graph graph, int start)
        {
            var visited = new HashSet<int>();
            
            if (!graph.AdjacencyList.ContainsKey(start))
                return visited;
                
            var queue = new Queue<int>();
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

        public Dictionary<int, int> GetLevelBFS(Graph graph, int start)
        {
            var visited = new HashSet<int>();
            var level = new Dictionary<int, int>
            {
                { start, 0 }
            };

            if (!graph.AdjacencyList.ContainsKey(start))
                return level;

            var queue = new Queue<int>();
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
}
