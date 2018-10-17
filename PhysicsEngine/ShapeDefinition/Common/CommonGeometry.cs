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

using SharpEngineMathUtility;
using System;
using System.Linq;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class CommonGeometry
    {
        #region Fields

        public Vector3d[] VerticesPosition { get; private set; }
        public SupportIndex[] VerticesIdx { get; private set; }
        public TriangleMesh[] Triangle { get; private set; }
        public CommonGeometry[] ConvexItem { get; private set; }
        
        #endregion

        #region Constructor

        public CommonGeometry(
            Vector3d[] vertexPosition,
            TriangleMesh[] triangle)
        {
            VerticesPosition = vertexPosition;
            Triangle = triangle;
            SetVertexAdjacency(Enumerable.Range(0, vertexPosition.Length).ToArray());
        }

        internal CommonGeometry(
            Vector3d[] vertexPosition,
            TriangleMesh[] triangle,
            int[] verticesIdx)
        {
            VerticesPosition = vertexPosition;
            Triangle = triangle;
            SetVertexAdjacency(verticesIdx);
        }

        #endregion

        #region Public Methods

        public void SetConcaveElements(CommonGeometry[] convexItems)
        {
            ConvexItem = convexItems;
        }

        #endregion

        #region Private Methods

        private void SetVertexAdjacency(int[] verticesIdx)
        {
            VerticesIdx = Array.ConvertAll(verticesIdx, x => new SupportIndex(x));

            if (Triangle != null)
            {
                var vList = VerticesIdx.ToList();

                foreach (var tr in Triangle)
                {
                    var indexA = vList.FindIndex(x => x.ID == tr.a);
                    var indexB = vList.FindIndex(x => x.ID == tr.b);
                    var indexC = vList.FindIndex(x => x.ID == tr.c);

                    vList[indexA].AddVertexToGlobalAdjList(tr.b);
                    vList[indexA].AddVertexToGlobalAdjList(tr.c);
                    vList[indexB].AddVertexToGlobalAdjList(tr.a);
                    vList[indexB].AddVertexToGlobalAdjList(tr.c);
                    vList[indexC].AddVertexToGlobalAdjList(tr.a);
                    vList[indexC].AddVertexToGlobalAdjList(tr.b);

                    vList[indexA].AddVertexToLocalAdjList(indexB);
                    vList[indexA].AddVertexToLocalAdjList(indexC);
                    vList[indexB].AddVertexToLocalAdjList(indexA);
                    vList[indexB].AddVertexToLocalAdjList(indexC);
                    vList[indexC].AddVertexToLocalAdjList(indexA);
                    vList[indexC].AddVertexToLocalAdjList(indexB);
                }

                VerticesIdx = vList.ToArray();
            }
        }

        #endregion

    }
}
