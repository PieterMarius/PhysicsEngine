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
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Helper
{
    internal static class CommonUtilities
    {
        public static VertexProperties GetVertexPosition(
            IGeometry obj,
            int vertexIndex)
        {
            return new VertexProperties(
                obj.Shape.Position +
                (obj.Shape.RotationMatrix * obj.Shape.VerticesRelPos[obj.VerticesIdx[vertexIndex].ID]),
                obj.VerticesIdx[vertexIndex].GetGlobalAdjacencyList());
        }

        public static TriangleMesh[] GetTriangleMeshes(int[][] inputTriangle)
        {
            TriangleMesh[] triangleMeshes = new TriangleMesh[inputTriangle.Length];

            for (int i = 0; i < inputTriangle.Length; i++)
                triangleMeshes[i] = new TriangleMesh(
                    inputTriangle[i][0],
                    inputTriangle[i][1],
                    inputTriangle[i][2]);

            return triangleMeshes;
        }

        public static Vector3d GetAABBMinValue(IGeometry[] geometry)
        {
            double xMin = double.MaxValue;
            double yMin = double.MaxValue;
            double zMin = double.MaxValue;

            foreach (var item in geometry)
            {
                if (item.AABBox.Min.x < xMin)
                    xMin = item.AABBox.Min.x;
                if (item.AABBox.Min.y < yMin)
                    yMin = item.AABBox.Min.y;
                if (item.AABBox.Min.z < zMin)
                    zMin = item.AABBox.Min.z;
            }

            return new Vector3d(xMin, yMin, zMin);
        }

        public static Vector3d GetAABBMaxValue(IGeometry[] geometry)
        {
            double xMax = double.MinValue;
            double yMax = double.MinValue;
            double zMax = double.MinValue;

            foreach (var item in geometry)
            {
                if (item.AABBox.Max.x > xMax)
                    xMax = item.AABBox.Max.x;
                if (item.AABBox.Max.y < yMax)
                    yMax = item.AABBox.Max.y;
                if (item.AABBox.Max.z < zMax)
                    zMax = item.AABBox.Max.z;
            }

            return new Vector3d(xMax, yMax, zMax);
        }
    }
}
