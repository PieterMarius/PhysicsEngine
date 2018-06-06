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
using SharpEngineMathUtility;
using SharpPhysicsEngine.ConvexHullWrapper;

namespace SharpPhysicsEngine.ShapeDefinition
{
	internal sealed class ConvexShape : Shape, IConvexShape
    {
        #region Object status properties
        
		public IGeometry ObjectGeometry { get; private set; }

        #endregion
        
        #region Constructor

        public ConvexShape(
            Vector3[] inputVertexPosition,
            TriangleMesh[] triangleMeshes,
            Vector3 position,
            double mass,
            bool isStatic)
        {
            ObjectType = ObjectType.RigidBody;

            ObjectGeometry = new Geometry(
                    this,
                    inputVertexPosition,
                    triangleMeshes,
                    ObjectGeometryType.ConvexShape,
                    true);

            SetIsStatic(isStatic);
            SetMass(mass);
            SetPosition(position);
            ShapeInit();
        }

        public ConvexShape(
            Vector3[] inputVertexPosition,
            IConvexHullEngine convexHullEngine,
            Vector3 position,
            double mass,
            bool isStatic)
        {
            ObjectType = ObjectType.RigidBody;
                        
            ConvexHullData convexHullData = convexHullEngine.GetConvexHull(inputVertexPosition);
            
            ObjectGeometry = new Geometry(
                    this,
                    convexHullData.Vertices,
                    convexHullData.TriangleMeshes,
                    ObjectGeometryType.ConvexShape,
                    true);

            SetIsStatic(isStatic);
            SetMass(mass);
            SetPosition(position);
            ShapeInit();
        }

        #endregion

        #region Public methods

        public override void SetAABB()
        {
            AABBox = AABB.GetGeometryAABB(ObjectGeometry, this);
            ObjectGeometry.SetAABB(AABBox);
        }

        public override void SetMass(double mass)
        {
            Mass = mass;

            if (IsStatic)
            {
                Mass = 0.0;
                InverseMass = 0.0;
            }
            else if (Mass > 0.0)
            {
                InverseMass = 1.0 / Mass;
                SetInertiaTensor();
            }
            else
                throw new Exception("Invalid mass value " + mass);
        }

        public override void Rotate(Vector3 versor, double angle)
        {
            var rotationQuaternion = new Quaternion(versor, angle);

            SetRotationStatus((rotationQuaternion * RotationStatus).Normalize());

            SetRotationMatrix(RotationStatus.ConvertToMatrix());

            SetInertiaTensor(
                (RotationMatrix * BaseInertiaTensor) *
                RotationMatrix.Transpose());
        }

        #endregion

        #region Private Methods

        private void ShapeInit()
        {
            SetObjectProperties();
            SetAABB();

            SleepingFrameCount = 0;
        }

        private void SetObjectProperties()
        {
            SetRotationMatrix();
            SetInertiaTensor();

            SetRelativePosition();
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
                Mass);

            Matrix3x3 baseTensors = ShapeCommonUtilities.GetInertiaTensor(
                    ObjectGeometry.VertexPosition,
                    ObjectGeometry.Triangle,
                    InitCenterOfMass,
                    Mass).InertiaTensor;

            BaseInertiaTensor = Matrix3x3.Invert(baseTensors);

            InertiaTensor = (RotationMatrix * BaseInertiaTensor) *
                             Matrix3x3.Transpose(RotationMatrix);
        }

        #endregion
    }
}

