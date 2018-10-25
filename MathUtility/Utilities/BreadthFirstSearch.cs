/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

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
