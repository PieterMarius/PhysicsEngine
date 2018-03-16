using System;
using System.Collections.Generic;

namespace SharpEngineMathUtility
{
    public static class BreadthFirstSearch
    {
        public static HashSet<int> BFS(Graph graph, int start)
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

        public static Dictionary<int, int> GetLevelBFS(Graph graph, int start)
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

        public static Dictionary<int, bool> GetBoolLevelBFS(Graph graph, int start)
        {
            var visited = new HashSet<int>();
            var level = new Dictionary<int, int>
            {
                { start, 0 }
            };

            var boolLevel = new Dictionary<int, bool>
            {
                { start, true }
            };

            if (!graph.AdjacencyList.ContainsKey(start))
                return boolLevel;

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
                        {
                            var sValue = value + 1;
                            level.Add(neighbor, sValue);
                            boolLevel.Add(neighbor, sValue % 2 == 0);
                        }
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return boolLevel;
        }
    }
}
