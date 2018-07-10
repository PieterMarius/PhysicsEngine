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
	internal class Geometry : IGeometry
	{
		#region Object Properties

		/// <summary>
		/// Vertex Position
		/// </summary>
		public SupportIndex[] VerticesIdx { get; private set; }
		/// <summary>
		/// Triangle Index
		/// </summary>
		public TriangleMesh[] Triangle { get; private set; }
		/// <summary>
		/// Bounding Box
		/// </summary>
		public AABB AABBox { get; private set; }
		/// <summary>
		/// Get the geometry property of object
		/// </summary>
		public ObjectGeometryType GeometryType { get; private set; }
		/// <summary>
		/// Pointer to belonging shape 
		/// </summary>
		public IShape Shape { get; private set; }

		#endregion

		#region Constructor

		public Geometry (
			IShape shape,
			int[] verticesIdx,
			TriangleMesh[] inputTriangle,
			ObjectGeometryType geometryType,
			bool getAdjacencyList)
		{
			Shape = shape;
			GeometryType = geometryType;

			if (inputTriangle != null)
			{
				Triangle = inputTriangle;
				
				SetVertexAdjacency(verticesIdx, getAdjacencyList);
			}
			else
			    SetVertexAdjacency(verticesIdx, getAdjacencyList);
		}

		public Geometry(
			IShape shape,
            int[] verticesIdx,
			ObjectGeometryType geometryType)
			: this(shape, verticesIdx, null, geometryType, false)
		{ }

		#endregion

		#region Public Methods
        	
		public void SetAABB(AABB box)
		{
			AABBox = box;
		}
        
        public Vector3[] GetVertices()
        {
            var result = new Vector3[VerticesIdx.Length];

            for (int i = 0; i < VerticesIdx.Length; i++)
                result[i] = Shape.Vertices[VerticesIdx[i].ID];

            return result;
        }

        #endregion

        #region Private Methods

        private void SetVertexAdjacency(
			int[] verticesIdx,
			bool getAdjacencyList)
		{
            VerticesIdx = Array.ConvertAll(verticesIdx, x => new SupportIndex(x));

            if (getAdjacencyList && Triangle != null)
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

