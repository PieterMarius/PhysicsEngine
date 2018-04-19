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

        public ConvexShape(ObjectType type)
        {
            ObjectType = type;

            InertiaTensor = Matrix3x3.IdentityMatrix();
            SleepingFrameCount = 0;
        }

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
                InverseMass = 1.0 / Mass;
        }

        public void SetGeometry(IGeometry geometry)
        {
            ObjectGeometry = geometry;
            
            SetObjectProperties();
            SetAABB();
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

        private void SetObjectProperties()
        {
            Matrix3x3 baseTensors = new Matrix3x3();

            int totalVertex = 0;

            StartPosition = CalculateMassCenter();
                        
            ////TODO Mass check
            Vector3[] vertexPosition = Array.ConvertAll(
                                    ObjectGeometry.VertexPosition,
                                    item => item.Vertex);

            var inertiaTensor = new InertiaTensor(
                    vertexPosition,
                    ObjectGeometry.Triangle,
                    Mass,
                    true);

            var normalizedInertiaTensor = inertiaTensor;

            totalVertex += ObjectGeometry.VertexPosition.Length;

            Vector3 r = inertiaTensor.GetMassCenter() - StartPosition;
            baseTensors += inertiaTensor.GetInertiaTensor() +
                            (Matrix3x3.IdentityMatrix() * r.Dot(r) - Matrix3x3.OuterProduct(r, r)) *
                            Mass;
            
            RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));

            SetRelativePosition(totalVertex);

            BaseInertiaTensor = Matrix3x3.Invert(baseTensors);
            InertiaTensor = (RotationMatrix * BaseInertiaTensor) *
                            Matrix3x3.Transpose(RotationMatrix);
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

        private Vector3 CalculateMassCenter()
        {
            Vector3[] vertexPosition = Array.ConvertAll(
                                        ObjectGeometry.VertexPosition,
                                        item => item.Vertex);

            var inertiaTensor = new InertiaTensor(
                    vertexPosition,
                    ObjectGeometry.Triangle,
                    Mass,
                    false);

            return inertiaTensor.GetMassCenter();
        }

        #endregion
    }
}

