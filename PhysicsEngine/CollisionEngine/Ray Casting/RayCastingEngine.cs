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
            bool selectShape = false)
        {
            var boundingBox = shape.AABBox;
            var hitBB = HitBoundingBox(
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

                            if(dst < distance)
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

        #endregion

        #region Private Methods

        private Vector3d? HitBoundingBox(
            Vector3d minB,
            Vector3d maxB,
            Vector3d origin,
            Vector3d dir)
        {
            bool inside = true;
            var quadrant = new int[3];
            int whichPlane = 0;
            var maxT = new double[3];
            var candidatePlane = new double[3];

            for (int i = 0; i < 3; i++)
            {
                if (origin[i] < minB[i])
                {
                    quadrant[i] = 1; //LEFT
                    candidatePlane[i] = minB[i];
                    inside = false;
                }
                else if (origin[i] > maxB[i])
                {
                    quadrant[i] = 0; //RIGHT
                    candidatePlane[i] = maxB[i];
                    inside = false;
                }
                else
                    quadrant[i] = 2; //MIDDLE
            }

            // Ray origin inside bounding box
            if (inside)
                return origin;

            // Calculate T distances to candidate plane
            for (int i = 0; i < 3; i++)
            {
                if (quadrant[i] != 2 && dir[i] != 0.0)
                    maxT[i] = (candidatePlane[i] - origin[i]) / dir[i];
                else
                    maxT[i] = -1;
            }
            
            // Get largest of the maxT's for final choice of intersection
            for (int i = 1; i < 3; i++)
            {
                if (maxT[whichPlane] < maxT[i])
                    whichPlane = i;
            }

            // Check final candidate actually inside box
            if (maxT[whichPlane] < 0.0)
                return null;

            var coord = new double[3];
            for (int i = 0; i < 3; i++)
            {
                if (whichPlane != i)
                {
                    coord[i] = origin[i] + maxT[whichPlane] * dir[i];
                    if (coord[i] < minB[i] || coord[i] > maxB[i])
                        return null;
                }
                else
                {
                    coord[i] = candidatePlane[i];
                }
            }
            
            // Ray hits box
            return new Vector3d(coord);
        }

        #endregion
    }
}
