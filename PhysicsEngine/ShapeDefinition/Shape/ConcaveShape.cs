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

using System;
using System.Linq;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ConvexHullWrapper;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal class ConcaveShape : Shape, IConcaveShape
    {
        #region Fields

        // <summary>
        /// Gometry of each convex shapes.
        /// </summary>
        /// <value>The object geometry.</value>
        public IGeometry[] ConvexShapesGeometry { get; private set; }

        /// <summary>
        /// Object geometry
        /// </summary>
        public IGeometry ObjectGeometry { get; private set; }

        /// <summary>
        /// Axis Aligned Bounding Box
        /// </summary>
        public AABB AABBox { get; private set; }

        /// <summary>
        /// Object triangle mesh index
        /// </summary>
        public TriangleMesh[] TriangleMeshes { get; private set; }
        
        /// <summary>
        /// Initial shape vertex position
        /// </summary>
        public Vector3[] InputVertexPosition { get; private set; }

        #endregion

        #region Constructor

        public ConcaveShape(
            TriangleMesh[] triangleMeshes,
            Vector3[] inputVertexPosition,
            IConvexHullEngine convexHullEngine,
            Vector3 position,
            double mass,
            bool isStatic)
        {
            ObjectType = ObjectType.RigidBody;
            Mass = mass;
            Position = position;
            TriangleMeshes = triangleMeshes;
            InputVertexPosition = inputVertexPosition;
            SetAABB();

            ObjectGeometry = new Geometry(
                this, 
                inputVertexPosition, 
                triangleMeshes, 
                ObjectGeometryType.ConcaveShape, 
                true);

            SetShapeGeometry(convexHullEngine);
        }

        #endregion

        #region Public Methods

        public override void Rotate(Vector3 versor, double angle)
        {
            throw new NotImplementedException();
        }

        public override void SetAABB()
        {
            AABBox = AABB.GetGeometryAABB(ObjectGeometry);
        }

        public override void SetMass(double mass)
        {
            Mass = mass;
        }

        #endregion

        #region Private Methods

        private void SetShapeGeometry(IConvexHullEngine convexHullEngine)
        {
            ShapeConvexDecomposition convexDecomposition = new ShapeConvexDecomposition(AABBox, TriangleMeshes);

            Vertex3Index[] verticesIndex = Array.ConvertAll(InputVertexPosition, x => new Vertex3Index(x, null, -1));
            var convexShapes = convexDecomposition.GetConvexShapeList(verticesIndex, 0.2);

            ConvexShapesGeometry = new Geometry[convexShapes.Count];

            for (int i = 0; i < convexShapes.Count; i++)
            {
                var convexVertices = Array.ConvertAll(convexShapes[i].Vertex3Idx.ToArray(), x => x.Vector3);

                ConvexHullData convexHullData = convexHullEngine.GetConvexHull(convexVertices);

                ConvexShapesGeometry[i] = new Geometry(
                    this,
                    convexHullData.Vertices,
                    convexHullData.TriangleMeshes,
                    ObjectGeometryType.ConvexShape,
                    true);
            }
        }

        #endregion
    }
}
