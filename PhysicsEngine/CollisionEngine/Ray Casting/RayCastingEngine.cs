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

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class RayCastingEngine
    {
        #region Constructor

        #endregion

        #region Public Methods

        public Vector3d? Execute(
            IShape shape,
            Vector3d origin, 
            Vector3d direction,
            bool selectShape)
        {
            var boundingBox = shape.AABBox;
            var hitBB = Helper.HitBoundingBox(
                boundingBox.Min,
                boundingBox.Max,
                origin,
                direction);

            if (hitBB.HasValue)
            {
                var vertices = Helper.GetVertexPosition1(shape);
                var triangles = Helper.GetTriangles(shape);
                Vector3d? result = null;

                for (int i = 0; i < vertices.Length; i++)
                {
                    var vt = vertices[i];
                    var distance = double.MaxValue;
                    foreach (var triangle in triangles[i])
                    {
                        var intersection = GeometryUtils.RayIntersectTriangle(
                                                    vt[triangle.a],
                                                    vt[triangle.b],
                                                    vt[triangle.c],
                                                    origin,
                                                    direction);

                        if (intersection.HasValue)
                        {
                            if (selectShape)
                                return intersection;

                            var dst = (origin - intersection.Value).Length();

                            if (dst < distance)
                            {
                                distance = dst;
                                result = intersection.Value;
                            }
                        }
                    }
                }

                return result;
            }
            
            return null;
        }

        public Vector3d? Execute(
            IShape[] shapes,
            Vector3d origin,
            Vector3d direction)
        {
            var minDistance = double.MaxValue;
            Vector3d? res = null;
            foreach (var shape in shapes)
            {
                var hit = Execute(shape, origin, direction, false);

                if (hit.HasValue)
                {
                    var dist = (origin - hit.Value).Length();
                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        res = hit.Value;
                    }
                }
            }

            return res;
        }

        public int? ExecuteWithID(
            IShape[] shapes,
            Vector3d origin,
            Vector3d direction)
        {
            var minDistance = double.MaxValue;
            int? res = null;
            foreach (var shape in shapes)
            {
                var hit = Execute(shape, origin, direction, true);

                if (hit.HasValue)
                {
                    var dist = (origin - hit.Value).Length();
                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        res = shape.ID;
                    }
                }
            }

            return res;
        }

        #endregion

        #region Private Methods

        #endregion
    }
}
