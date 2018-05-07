﻿/******************************************************************************
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
using SharpEngineMathUtility;
using SharpPhysicsEngine.Helper;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class CompoundShape : Shape, IShapeCommon, ICompoundShape, IDentity
    {
        #region Object status properties
                
        /// <summary>
        /// ObjectGeometry Masses
        /// </summary>
        public double[] PartialMass { get; private set; }
        
        /// <summary>
        /// Gets or sets the object geometry.
        /// </summary>
        /// <value>The object geometry.</value>
        public IGeometry[] ObjectGeometry { get; private set; }
        
        /// <summary>
        /// Get the number of convex objects made the main object.
        /// </summary>
        public int CompoundingConvexObjCount { get; private set; }

        #endregion

        #region Object dynamic properties
                
        /// <summary>
        /// Gets the start position of each elements of composite Objects
        /// </summary>
        public Vector3[] StartCompoundPositionObjects { get; private set; }
        
        #endregion
        
        #region Constructor

        public CompoundShape(
            List<Vector3[]> inputVertexPosition,
            List<int[][]> inputTriangle,
            Vector3[] compoundPosition,
            double[] mass)
        {
            ObjectType = ObjectType.CompoundShape;

            if (IsStatic)
            {
                Mass = 0.0;
                InverseMass = 0.0;
            }
            else if (Mass > 0.0)
                InverseMass = 1.0;

            InertiaTensor = Matrix3x3.IdentityMatrix();
            SleepingFrameCount = 0;
            StartCompoundPositionObjects = compoundPosition;
            SetPartialMass(mass);

            IGeometry[] geometry = new IGeometry[inputVertexPosition.Count];

            for (int i = 0; i < inputVertexPosition.Count; i++)
            {
                TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(inputTriangle[i]);

                geometry[i] = new Geometry(this, inputVertexPosition[i], triangleMeshes, ObjectGeometryType.ConvexShape, true);
            }

            SetObjectGeometry(geometry);
        }

        #endregion

        #region Public methods
        
        public override void SetAABB()
        {
            if (ObjectGeometry.Length == 1)
                ObjectGeometry[0].SetAABB(AABB.GetGeometryAABB(ObjectGeometry[0]));
            else if (ObjectGeometry.Length > 1)
            {
                int geometryIndex = 0;
                foreach (Geometry obj in ObjectGeometry)
                {
                    obj.SetAABB(AABB.GetGeometryAABB(obj));
                    geometryIndex++;
                }
            }
        }

        public override void SetMass(double mass)
        {
            Mass = mass;
            if (Mass > 0.0)
                InverseMass = 1.0 / Mass;
        }

        public void SetPartialMass(double[] mass)
        {
            PartialMass = new double[mass.Length];
            Array.Copy(mass, PartialMass, mass.Length);

            for (int i = 0; i < mass.Length; i++)
                Mass += mass[i];

            if (IsStatic)
            {
                Mass = 0.0;
                InverseMass = 0.0;
            }
            else if (Mass > 0.0)
                InverseMass = 1.0 / Mass;
        }

        public void SetCompoundPosition(Vector3[] compoundPosition)
        {
            StartCompoundPositionObjects = compoundPosition;
        }

        public override void Rotate(Vector3 versor, double angle)
        {
            throw new NotImplementedException();
        }

        #endregion

        #region Private Methods

        private void SetObjectGeometry(IGeometry[] geometry)
        {
            ObjectGeometry = geometry;

            if (geometry != null)
            {
                for (int i = 0; i < ObjectGeometry.Length; i++)
                {
                    if (ObjectGeometry[i].VertexPosition != null &&
                        ObjectGeometry[i].VertexPosition.Length > 0)
                    {
                        for (int j = 0; j < ObjectGeometry[i].VertexPosition.Length; j++)
                            ObjectGeometry[i].VertexPosition[j].SetVertexPosition(ObjectGeometry[i].VertexPosition[j].Vertex +
                                                                                  StartCompoundPositionObjects[i]);
                    }
                }
                CompoundingConvexObjCount = ObjectGeometry.Length;
            }

            SetObjectProperties();
            SetAABB();
        }

        private void SetObjectProperties()
        {
            Matrix3x3 baseTensors = new Matrix3x3();

            int totalVertex = 0;

            InitCenterOfMass = CalculateCenterOfMass();

            for (int i = 0; i < ObjectGeometry.Length; i++)
            {
                baseTensors += ShapeCommonUtilities.GetInertiaTensor(
                    ObjectGeometry[i].VertexPosition,
                    ObjectGeometry[i].Triangle,
                    InitCenterOfMass,
                    PartialMass[i]);

                Vector3[] vertexPosition = Array.ConvertAll(
                                        ObjectGeometry[i].VertexPosition,
                                        item => item.Vertex);

                totalVertex += ObjectGeometry[i].VertexPosition.Length;
            }

            RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));

            SetRelativePosition(totalVertex);

            BaseInertiaTensor = Matrix3x3.Invert(baseTensors);
            InertiaTensor = (RotationMatrix * BaseInertiaTensor) *
                            Matrix3x3.Transpose(RotationMatrix);
        }

        private void SetRelativePosition(int totalVertex)
        {
            for (int i = 0; i < ObjectGeometry.Length; i++)
            {
                Vector3[] relativePositions = new Vector3[ObjectGeometry[i].VertexPosition.Length];
                if (ObjectGeometry[i].VertexPosition.Length > 0)
                {
                    for (int j = 0; j < ObjectGeometry[i].VertexPosition.Length; j++)
                        relativePositions[j] =
                            ObjectGeometry[i].VertexPosition[j].Vertex -
                            InitCenterOfMass;
                }

                ObjectGeometry[i].SetRelativePosition(relativePositions);
            }
        }

        private Vector3 CalculateCenterOfMass()
        {
            Vector3 startPosition = new Vector3();

            for (int i = 0; i < ObjectGeometry.Length; i++)
            {
                Vector3[] vertices = Array.ConvertAll(
                                        ObjectGeometry[i].VertexPosition,
                                        item => item.Vertex);

                var centerOfMass = ShapeCommonUtilities.CalculateCenterOfMass(
                    vertices,
                    ObjectGeometry[i].Triangle,
                    PartialMass[i]);
                                
                startPosition += centerOfMass * PartialMass[i];
            }

            if (Mass > 0.0)
                return startPosition / Mass;

            return Vector3.ToZero();
        }

        #endregion
    }
}
