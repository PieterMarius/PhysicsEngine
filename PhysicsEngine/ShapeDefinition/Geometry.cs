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
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;

namespace SharpPhysicsEngine.ShapeDefinition
{
	internal class Geometry : IGeometry
	{
		#region Object Properties

		/// <summary>
		/// Vertex Position
		/// </summary>
		public VertexProperties[] VertexPosition { get; private set; }
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
		/// Relative position respect mass center
		/// </summary>
		public Vector3[] RelativePosition { get; private set; }
		/// <summary>
		/// Pointer to belonging shape 
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
			VertexPosition = new VertexProperties[inputVertexPosition.Length];

			Parallel.For(0,
				VertexPosition.Length,
				new ParallelOptions { MaxDegreeOfParallelism = 4 },
				i =>
				{
					VertexPosition[i] = new VertexProperties(inputVertexPosition[i]);

					if (Triangle != null && 
						getAdjacencyList)
					{
						HashSet<int> adjacencyList = new HashSet<int>();

						for (int j = 0; j < Triangle.Length; j++)
						{
							if (Triangle[j].a == i)
							{
								adjacencyList.Add(Triangle[j].b);
								adjacencyList.Add(Triangle[j].c);
								continue;
							}
							if (Triangle[j].b == i)
							{
								adjacencyList.Add(Triangle[j].a);
								adjacencyList.Add(Triangle[j].c);
								continue;
							}
							if (Triangle[j].c == i)
							{
								adjacencyList.Add(Triangle[j].a);
								adjacencyList.Add(Triangle[j].b);
								continue;
							}
						}
						VertexPosition[i].SetAdjacencyList(adjacencyList.ToList());
					}
				});
		}

		#endregion
	}
}

