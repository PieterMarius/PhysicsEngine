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
            bool isStatic) : base()
        {
            ObjectType = ObjectType.RigidBody;
            MassInfo.Mass = mass;
            Position = position;
            TriangleMeshes = triangleMeshes;
            InputVertexPosition = inputVertexPosition;
            
            ObjectGeometry = new Geometry(
                this, 
                inputVertexPosition, 
                triangleMeshes, 
                ObjectGeometryType.ConcaveShape, 
                true);

            SetRotationMatrix();
            SetInertiaTensor();
                        
            SetShapeGeometry(convexHullEngine);
            SetRelativePosition();
            SetShapesRelativePosition();
            SetAABB();
        }

        #endregion

        #region Public Methods

        public override void Rotate(Vector3 versor, double angle)
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
            
            foreach (var shape in ConvexShapesGeometry)
                shape.SetAABB(AABB.GetGeometryAABB(shape, shape));
        }

        public override void SetMass(double mass)
        {
            MassInfo.Mass = mass;
        }

        #endregion

        #region Private Methods

        private void SetShapeGeometry(IConvexHullEngine convexHullEngine)
        {
            AABB region = AABB.GetGeometryAABB(Array.ConvertAll(ObjectGeometry.VertexPosition, x => x.Vertex), this);

            ShapeConvexDecomposition convexDecomposition = new ShapeConvexDecomposition(region, TriangleMeshes);

            Vertex3Index[] verticesIndex = Array.ConvertAll(ObjectGeometry.VertexPosition, x => new Vertex3Index(x.Vertex, x.GetAdjacencyList(), -1));
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

        private void SetShapesRelativePosition()
        {
            List<Vector3> objectRelativePosition = new List<Vector3>();
            double dist = 0.0;

            for (int i = 0; i < ConvexShapesGeometry.Length; i++)
            {
                Vector3[] relativePositions = new Vector3[ConvexShapesGeometry[i].VertexPosition.Length];

                if (ConvexShapesGeometry[i].VertexPosition.Length > 0)
                {
                    for (int j = 0; j < ConvexShapesGeometry[i].VertexPosition.Length; j++)
                    {
                        relativePositions[j] =
                            ConvexShapesGeometry[i].VertexPosition[j].Vertex -
                            InitCenterOfMass;

                        objectRelativePosition.Add(relativePositions[j]);

                        double length = relativePositions[j].Dot(relativePositions[j]);

                        if (length > dist)
                        {
                            dist = length;
                            FarthestPoint = relativePositions[j];
                        }
                    }
                }

                ConvexShapesGeometry[i].SetRelativePosition(relativePositions);
            }
        }

        private void SetRelativePosition()
        {
            Vector3[] relativePositions = new Vector3[ObjectGeometry.VertexPosition.Length];
            double dist = 0.0;

            if (ObjectGeometry.VertexPosition.Length > 0)
            {
                for (int j = 0; j < ObjectGeometry.VertexPosition.Length; j++)
                {
                    relativePositions[j] =
                        ObjectGeometry.VertexPosition[j].Vertex -
                        InitCenterOfMass;

                    double length = relativePositions[j].Dot(relativePositions[j]);

                    if (length > dist)
                    {
                        dist = length;
                        FarthestPoint = relativePositions[j];
                    }
                }
            }

            ObjectGeometry.SetRelativePosition(relativePositions);
        }

        private void SetRotationMatrix()
        {
            RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));
        }

        private void SetInertiaTensor()
        {
            Vector3[] vertices = Array.ConvertAll(
                                        ObjectGeometry.VertexPosition,
                                        item => item.Vertex);

            InitCenterOfMass = ShapeCommonUtilities.CalculateCenterOfMass(
                vertices,
                ObjectGeometry.Triangle,
                MassInfo.Mass);

            Matrix3x3 baseTensors = ShapeCommonUtilities.GetInertiaTensor(
                    ObjectGeometry.VertexPosition,
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
