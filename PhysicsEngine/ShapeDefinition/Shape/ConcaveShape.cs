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
using System.Collections.Generic;
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
        /// Object triangle mesh index
        /// </summary>
        public TriangleMesh[] TriangleMeshes { get; private set; }
        
        #endregion

        #region Constructor

        public ConcaveShape(
            TriangleMesh[] triangleMeshes,
            Vector3d[] inputVertexPosition,
            IConvexHullEngine convexHullEngine,
            Vector3d position,
            double mass,
            bool isStatic) : base()
        {
            ObjectType = ObjectType.RigidBody;
            SetIsStatic(isStatic);
            
            Position = position;
            TriangleMeshes = triangleMeshes;
            Vertices = inputVertexPosition;
            
            ObjectGeometry = new Geometry(
                this, 
                Enumerable.Range(0, inputVertexPosition.Length).ToArray(), 
                triangleMeshes, 
                ObjectGeometryType.ConcaveShape, 
                true);

            SetRotationMatrix();
            SetMass(mass);
            
            SetShapeGeometry(convexHullEngine);
            SetRelativePosition();
            SetAABB();
        }

        #endregion

        #region Public Methods

        public override void Rotate(Vector3d versor, double angle)
        {
            var rotationQuaternion = new Quaternion(versor, angle);

            SetRotationStatus((rotationQuaternion * RotationStatus).Normalize());

            SetRotationMatrix(RotationStatus.ConvertToMatrix());

            SetInverseInertiaTensor(
                (RotationMatrix * MassInfo.InverseBaseInertiaTensor) *
                RotationMatrix.Transpose());
        }

        public override void SetAABB()
        {
            AABBox = AABB.GetGeometryAABB(ObjectGeometry, this);
            ObjectGeometry.SetAABB(AABBox);
            
            foreach (var shape in ConvexShapesGeometry)
                shape.SetAABB(AABB.GetGeometryAABB(shape, shape));
        }

        public override void SetMass(double mass)
        {
            MassInfo.Mass = mass;

            if (IsStatic)
            {
                MassInfo.Mass = 0.0;
                MassInfo.InverseMass = 0.0;
            }
            else if (MassInfo.Mass > 0.0)
            {
                MassInfo.InverseMass = 1.0 / MassInfo.Mass;
                SetInertiaTensor();
            }
            else
                throw new Exception("Invalid mass value " + mass);
        }

        #endregion

        #region Private Methods

        private void SetShapeGeometry(IConvexHullEngine convexHullEngine)
        {
            AABB region = AABB.GetGeometryAABB(Vertices, this);
                        
            Vertex3Index[] verticesIndex = new Vertex3Index[Vertices.Length];

            for (int i = 0; i < Vertices.Length; i++)
                verticesIndex[i] = new Vertex3Index(Vertices[i], ObjectGeometry.VerticesIdx[i].GetGlobalAdjacencyList(), i);

            ConvexDecompositionEngine convexDecomposition = new ConvexDecompositionEngine(region, verticesIndex, 0.2);

            var convexShapes = convexDecomposition.Execute().GetConvexShapeList(true);

            ConvexShapesGeometry = new Geometry[convexShapes.Count];

            for (int i = 0; i < convexShapes.Count; i++)
            {
                ConvexHullData convexHullData = convexHullEngine.GetConvexHull(convexShapes[i].Vertex3Idx.ToArray());

                var verticesIdx = Array.ConvertAll(convexHullData.Vertices, x => x.ID);

                ConvexShapesGeometry[i] = new Geometry(
                    this,
                    verticesIdx,
                    convexHullData.TriangleMeshes,
                    ObjectGeometryType.ConvexShape,
                    true);
            }
        }

        private void SetRelativePosition()
        {
            VerticesRelPos = new Vector3d[Vertices.Length];
            double dist = double.MinValue;

            for (int i = 0; i < Vertices.Length; i++)
            {
                var rPos = Vertices[i] - InitCenterOfMass;
                VerticesRelPos[i] = rPos;
                double length = rPos.Dot(rPos);

                if (length > dist)
                {
                    dist = length;
                    FarthestPoint = rPos;
                }
            }

        }

        private void SetRotationMatrix()
        {
            RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));
        }

        private void SetInertiaTensor()
        {
            InitCenterOfMass = ShapeCommonUtilities.CalculateCenterOfMass(
                Vertices,
                ObjectGeometry.Triangle,
                MassInfo.Mass);

            Matrix3x3 baseTensors = ShapeCommonUtilities.GetInertiaTensor(
                    Vertices,
                    ObjectGeometry.Triangle,
                    InitCenterOfMass,
                    MassInfo.Mass).InertiaTensor;

            MassInfo.InverseBaseInertiaTensor = Matrix3x3.Invert(baseTensors);

            MassInfo.InverseInertiaTensor = (RotationMatrix * MassInfo.InverseBaseInertiaTensor) *
                                             Matrix3x3.Transpose(RotationMatrix);
        }
        
        #endregion
    }
}
