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
using SharpPhysicsEngine.ConvexHullWrapper;
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Linq;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class ShapeGeometry
    {
        #region Fields

        private readonly CommonGeometry Geometry;

        #endregion

        #region Constructor

        public ShapeGeometry(
            Vector3d[] inputVertexPosition,
            int[][] inputTriangle)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(inputTriangle);
            Geometry = new CommonGeometry(inputVertexPosition, triangleMeshes);
        }

        public ShapeGeometry(Vector3d[] inputVertexPosition)
        {
            IConvexHullEngine convexHullEngine = new ConvexHullEngine();

            ConvexHullData convexHullData = convexHullEngine.GetConvexHull(inputVertexPosition);
            TriangleMesh[] triangleMeshes = convexHullData.TriangleMeshes;
            Geometry = new CommonGeometry(
                Array.ConvertAll(convexHullData.Vertices, x => x.Vector3), 
                triangleMeshes);
        }

        #endregion

        #region Public Methods

        internal CommonGeometry GetGeometry()
        {
            return Geometry;
        }

        internal void SetConcaveCommonGeometry()
        {
            if (Geometry.ConvexItem == null)
            {
                IConvexHullEngine convexHullEngine = new ConvexHullEngine();
                Geometry.SetConcaveElements(GetShapeGeometry(convexHullEngine));
            }
        }

        #endregion

        #region Private Methods

        private CommonGeometry[] GetShapeGeometry(IConvexHullEngine convexHullEngine)
        {
            AABB region = AABB.GetGeometryAABB(Geometry.VerticesPosition, this);

            Vertex3Index[] verticesIndex = new Vertex3Index[Geometry.VerticesPosition.Length];

            for (int i = 0; i < Geometry.VerticesPosition.Length; i++)
                verticesIndex[i] = new Vertex3Index(Geometry.VerticesPosition[i], Geometry.VerticesIdx[i].GetGlobalAdjacencyList(), i);

            ConvexDecompositionEngine convexDecomposition = new ConvexDecompositionEngine(region, verticesIndex, 0.2);
            var convexShapes = convexDecomposition.Execute().GetConvexShapeList(true);
            var ConvexShapesGeometry = new CommonGeometry[convexShapes.Count];

            for (int i = 0; i < convexShapes.Count; i++)
            {
                ConvexHullData convexHullData = convexHullEngine.GetConvexHull(convexShapes[i].Vertex3Idx.ToArray());

                var verticesIdx = Array.ConvertAll(convexHullData.Vertices, x => x.ID);
                ConvexShapesGeometry[i] = new CommonGeometry(null, convexHullData.TriangleMeshes, verticesIdx);
            }

            return ConvexShapesGeometry;
        }

        #endregion
    }
}
