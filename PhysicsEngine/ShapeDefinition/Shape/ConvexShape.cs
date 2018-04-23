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

namespace SharpPhysicsEngine.ShapeDefinition
{
	internal sealed class ConvexShape : Shape, IShapeCommon, IConvexShape
    {
        #region Object status properties
        
		public IGeometry ObjectGeometry { get; private set; }

        #endregion
        
        #region Constructor

        public ConvexShape(
            TriangleMesh[] triangleMeshes,
            Vector3[] inputVertexPosition,
            Vector3 position,
            double mass,
            bool isStatic)
        {
            ObjectType = ObjectType.RigidBody;

            ObjectGeometry = new Geometry(
                    this,
                    inputVertexPosition,
                    triangleMeshes,
                    ObjectGeometryType.ConvexBody,
                    true);

            SetIsStatic(isStatic);
            SetMass(mass);
            SetPosition(position);
            ShapeInit();
        }

        public ConvexShape(
            TriangleMesh[] triangleMeshes,
            Vector3[] inputVertexPosition,
            Vector3 position,
            double mass)
            : this(triangleMeshes, inputVertexPosition, position, mass, false)
        { }

        #endregion

        #region Public methods

        public override void SetAABB()
        {
            if (ObjectGeometry != null)
                ObjectGeometry.SetAABB(AABB.GetGeometryAABB(ObjectGeometry));
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

            SetRelativePosition(ObjectGeometry.VertexPosition.Length);
        }

        private void SetRelativePosition(int totalVertex)
        {
            Vector3[] relativePositions = new Vector3[ObjectGeometry.VertexPosition.Length];
            if (ObjectGeometry.VertexPosition.Length > 0)
            {
                for (int j = 0; j < ObjectGeometry.VertexPosition.Length; j++)
                    relativePositions[j] =
                        ObjectGeometry.VertexPosition[j].Vertex -
                        StartPosition;
            }

            ObjectGeometry.SetRelativePosition(relativePositions);
        }

        private void SetRotationMatrix()
        {
            RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));
        }

        private void SetInertiaTensor()
        {
            StartPosition = ShapeCommonUtilities.CalculateCenterOfMass(
                ObjectGeometry.VertexPosition,
                ObjectGeometry.Triangle,
                Mass);

            Matrix3x3 baseTensors = ShapeCommonUtilities.GetInertiaTensor(
                    ObjectGeometry.VertexPosition,
                    ObjectGeometry.Triangle,
                    StartPosition,
                    Mass);

            BaseInertiaTensor = Matrix3x3.Invert(baseTensors);

            InertiaTensor = (RotationMatrix * BaseInertiaTensor) *
                            Matrix3x3.Transpose(RotationMatrix);
        }

        #endregion
    }
}

