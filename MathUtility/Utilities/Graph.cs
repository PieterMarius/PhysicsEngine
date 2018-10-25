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
