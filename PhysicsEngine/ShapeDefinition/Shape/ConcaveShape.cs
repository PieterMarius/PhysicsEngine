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
        /// Vertices position
        /// </summary>
        public override Vector3d[] Vertices { get { return ObjectGeometry.BaseGeometry.VerticesPosition; } }

        #endregion

        #region Constructor

        public ConcaveShape(
            CommonGeometry baseGeometry,
            Vector3d position,
            double mass,
            bool isStatic) : base()
        {
            ObjectType = ObjectType.RigidBody;
            SetIsStatic(isStatic);
            
            Position = position;
                                  
            ObjectGeometry = new Geometry(
                this, 
                baseGeometry,
                ObjectGeometryType.ConcaveShape, 
                true);

            SetRotationMatrix();
            SetMass(mass);
            
            SetShapeGeometry();
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

        private void SetShapeGeometry()
        {
            ConvexShapesGeometry = new Geometry[ObjectGeometry.BaseGeometry.ConvexItem.Length];
            for (int i = 0; i < ObjectGeometry.BaseGeometry.ConvexItem.Length; i++)
            {
                ConvexShapesGeometry[i] = new Geometry(
                    this,
                    ObjectGeometry.BaseGeometry.ConvexItem[i],
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
                ObjectGeometry.BaseGeometry.Triangle,
                MassInfo.Mass);

            Matrix3x3 baseTensors = ShapeCommonUtilities.GetInertiaTensor(
                    Vertices,
                    ObjectGeometry.BaseGeometry.Triangle,
                    InitCenterOfMass,
                    MassInfo.Mass).InertiaTensor;

            MassInfo.InverseBaseInertiaTensor = Matrix3x3.Invert(baseTensors);

            MassInfo.InverseInertiaTensor = (RotationMatrix * MassInfo.InverseBaseInertiaTensor) *
                                             Matrix3x3.Transpose(RotationMatrix);
        }
        
        #endregion
    }
}
