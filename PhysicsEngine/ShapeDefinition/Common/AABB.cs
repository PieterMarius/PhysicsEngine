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
using SharpPhysicsEngine.NonConvexDecomposition.Octree;

namespace SharpPhysicsEngine.ShapeDefinition
{
	internal class AABB
	{
		#region Object fields

		public Vector3 Min;
		public Vector3 Max;
		public bool positionAABBChanged { get; private set; }

		#endregion

		#region Constructor

		public AABB (
			double minX,
			double maxX,
			double minY,
			double maxY,
			double minZ,
			double maxZ,
			bool positionChanged)
		{
			Min = new Vector3(minX, minY, minZ);
			Max = new Vector3(maxX, maxY, maxZ);
			positionAABBChanged = positionChanged;
		}

		public AABB(Vector3 min, Vector3 max)
		{
			Min = min;
			Max = max;
		}

		#endregion

		#region Public methods

		public void SetPositionChanged(bool value)
		{
			positionAABBChanged = value; 
		}

		public bool Contains(AABB box)
		{
			return Min.x <= box.Min.x && Max.x >= box.Max.x &&
				   Min.y <= box.Min.y && Max.y >= box.Max.y &&
				   Min.z <= box.Min.z && Max.z >= box.Max.z;
		}

		public bool Contains(Vector3 point)
		{
			return point.x >= Min.x && point.x <= Max.x &&
				   point.y >= Min.y && point.y <= Max.y &&
				   point.z >= Min.z && point.z <= Max.z;
		}

		public bool Intersect(AABB box)
		{
			return Max.x > box.Min.x &&
				   Min.x < box.Max.x &&
				   Max.y > box.Min.y &&
				   Min.y < box.Max.y &&
				   Max.z > box.Min.z &&
				   Min.z < box.Max.z;
		}

		public static bool Intersect(
			AABB a,
			AABB b,
			double distanceTolerance)
		{
			return a.Min[0] - b.Max[0] <= distanceTolerance &&
				   a.Max[0] - b.Min[0] >= -distanceTolerance &&
				   a.Min[1] - b.Max[1] <= distanceTolerance &&
				   a.Max[1] - b.Min[1] >= -distanceTolerance &&
				   a.Min[2] - b.Max[2] <= distanceTolerance &&
				   a.Max[2] - b.Min[2] >= -distanceTolerance;
		}

        public bool Intersect(
            AABB box,
            double distanceTolerance)
        {
            return Min[0] - box.Max[0] <= distanceTolerance &&
                   Max[0] - box.Min[0] >= -distanceTolerance &&
                   Min[1] - box.Max[1] <= distanceTolerance &&
                   Max[1] - box.Min[1] >= -distanceTolerance &&
                   Min[2] - box.Max[2] <= distanceTolerance &&
                   Max[2] - box.Min[2] >= -distanceTolerance;
        }

        #endregion

        #region Public static methods

        public static AABB GetGeometryAABB(IGeometry simObject)
		{
			Vector3 vertexPos = Helper.GetVertexPosition(simObject, 0);
			double xMax = vertexPos.x;
			double xMin = vertexPos.x;
			double yMax = vertexPos.y;
			double yMin = vertexPos.y;
			double zMax = vertexPos.z;
			double zMin = vertexPos.z;

			for (int i = 1; i < simObject.RelativePosition.Length; i++)
			{
				Vector3 vertex = Helper.GetVertexPosition(simObject, i);

				if (vertex.x < xMin)
					xMin = vertex.x;
				else if (vertex.x > xMax)
					xMax = vertex.x;

				if (vertex.y < yMin)
					yMin = vertex.y;
				else if (vertex.y > yMax)
					yMax = vertex.y;

				if (vertex.z < zMin)
					zMin = vertex.z;
				else if (vertex.z > zMax)
					zMax = vertex.z;
			}

			return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
		}

        public static AABB GetGeometryAABB(Vector3[] vertices)
        {
            Vector3 vertexPos = vertices[0];
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < vertices.Length; i++)
            {
                Vector3 vertex = vertices[i];

                if (vertex.x < xMin)
                    xMin = vertex.x;
                else if (vertex.x > xMax)
                    xMax = vertex.x;

                if (vertex.y < yMin)
                    yMin = vertex.y;
                else if (vertex.y > yMax)
                    yMax = vertex.y;

                if (vertex.z < zMin)
                    zMin = vertex.z;
                else if (vertex.z > zMax)
                    zMax = vertex.z;
            }

            return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
        }

        public static AABB GetShapePointAABB(SoftShapePoint[] shapePoint)
		{
			Vector3 vertexPos = shapePoint[0].Position;
			double xMax = vertexPos.x;
			double xMin = vertexPos.x;
			double yMax = vertexPos.y;
			double yMin = vertexPos.y;
			double zMax = vertexPos.z;
			double zMin = vertexPos.z;

			for (int i = 1; i < shapePoint.Length; i++)
			{
				Vector3 vertex = shapePoint[i].Position;

				if (vertex.x < xMin)
					xMin = vertex.x;
				else if (vertex.x > xMax)
					xMax = vertex.x;

				if (vertex.y < yMin)
					yMin = vertex.y;
				else if (vertex.y > yMax)
					yMax = vertex.y;

				if (vertex.z < zMin)
					zMin = vertex.z;
				else if (vertex.z > zMax)
					zMax = vertex.z;
			}

			return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
		}

		public static AABB GetTriangleAABB(Vector3[] triangle)
		{
			Vector3 vertexPos = triangle[0];
			double xMax = vertexPos.x;
			double xMin = vertexPos.x;
			double yMax = vertexPos.y;
			double yMin = vertexPos.y;
			double zMax = vertexPos.z;
			double zMin = vertexPos.z;

			for (int i = 1; i < triangle.Length; i++)
			{
				Vector3 vertex = triangle[i];

				if (vertex.x < xMin)
					xMin = vertex.x;
				else if (vertex.x > xMax)
					xMax = vertex.x;

				if (vertex.y < yMin)
					yMin = vertex.y;
				else if (vertex.y > yMax)
					yMax = vertex.y;

				if (vertex.z < zMin)
					zMin = vertex.z;
				else if (vertex.z > zMax)
					zMax = vertex.z;
			}

			return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
		}
		
		#endregion
	}
}

