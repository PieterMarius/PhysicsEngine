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

using SharpPhysicsEngine.ConvexHullWrapper;
using SharpEngineMathUtility;
using System.Linq;
using System;

namespace SharpPhysicsEngine.Wrapper
{
    public static class PhysicsEngineUtilities
    {
        #region Fields

        public struct ConvexHull
        {
            public double[][] Vertices;
            public int[][] Triangles;
        }

        #endregion

        #region Public Methods

        public static ConvexHull ExtractConvexHull(double[][] vertex)
        {
            IConvexHullEngine convexHullEngine = new ConvexHullEngine();

            var inputVertex = MathUtils.GetVector3ArrayFromMatrix(vertex);

            ConvexHullData convexHullData = convexHullEngine.GetConvexHull(inputVertex);

            var result = new ConvexHull()
            {
                Triangles = convexHullData.TriangleMeshes.Select(x => x.GetArray())?.ToArray(),
                Vertices = MathUtils.GetArrayFromVector3(Array.ConvertAll(convexHullData.Vertices, x => x.Vector3))
            };
                        
            return result;
        }

        #endregion
    }
}
