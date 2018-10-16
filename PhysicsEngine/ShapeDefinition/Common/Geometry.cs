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
        /// Base Geometry
        /// </summary>
        public CommonGeometry BaseGeometry { get; private set; }
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
			CommonGeometry baseGeometry,
			ObjectGeometryType geometryType,
			bool getAdjacencyList)
		{
			Shape = shape;
			GeometryType = geometryType;
            BaseGeometry = baseGeometry;
		}

		public Geometry(
			IShape shape,
            CommonGeometry baseGeometry,
			ObjectGeometryType geometryType)
			: this(shape, baseGeometry, geometryType, false)
		{ }
                
		#endregion

		#region Public Methods
        	
		public void SetAABB(AABB box)
		{
			AABBox = box;
		}
        
        public Vector3d[] GetVertices()
        {
            var result = new Vector3d[BaseGeometry.VerticesIdx.Length];

            for (int i = 0; i < BaseGeometry.VerticesIdx.Length; i++)
                result[i] = Shape.Vertices[BaseGeometry.VerticesIdx[i].ID];

            return result;
        }

        public AABB GetAABB()
        {
            return AABBox;
        }

        public int CompareTo(object obj)
        {
            return obj.GetHashCode().CompareTo(GetHashCode());
        }

        #endregion

        #region Private Methods
        
        #endregion
    }
}

