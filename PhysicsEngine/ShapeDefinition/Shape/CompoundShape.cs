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
using SharpPhysicsEngine.Helper;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class CompoundShape : Shape, ICompoundShape, IDentity
    {
        #region Object status properties
                
        /// <summary>
        /// ObjectGeometry Masses
        /// </summary>
        public double[] PartialMass { get; private set; }

        /// <summary>
        /// Gometry of each convex shapes.
        /// </summary>
        /// <value>The object geometry.</value>
        public IGeometry[] ShapesGeometry { get; private set; }

        /// <summary>
        /// Object geometry
        /// </summary>
        public IGeometry ObjectGeometry { get; private set; }

        /// <summary>
        /// Get the number of convex objects made the main object.
        /// </summary>
        public int CompoundingConvexObjCount { get; private set; }

        /// <summary>
        /// Object triangle mesh index
        /// </summary>
        public TriangleMesh[] TriangleMeshes { get; private set; }
                
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
            double[] mass) : base()
        {
            ObjectType = ObjectType.CompoundShape;

            if (IsStatic)
            {
                MassInfo.Mass = 0.0;
                MassInfo.InverseMass = 0.0;
            }
            else if (MassInfo.Mass > 0.0)
                MassInfo.InverseMass = 1.0;
            
            MassInfo.InverseInertiaTensor = Matrix3x3.IdentityMatrix();
            SleepingFrameCount = 0;
            StartCompoundPositionObjects = compoundPosition;
            SetPartialMass(mass);
                                                
            SetShapesGeometry(GetShapesGeometry(inputVertexPosition, inputTriangle));

        }

        #endregion

        #region Public methods
        
        public override void SetAABB()
        {
            if (ShapesGeometry.Length == 1)
                ShapesGeometry[0].SetAABB(AABB.GetGeometryAABB(ShapesGeometry[0], ShapesGeometry[0]));
            else if (ShapesGeometry.Length > 1)
            {
                int geometryIndex = 0;
                foreach (Geometry obj in ShapesGeometry)
                {
                    obj.SetAABB(AABB.GetGeometryAABB(obj, obj));
                    geometryIndex++;
                }
            }
            //ObjectGeometry.SetAABB(AABB.GetGeometryAABB(ObjectGeometry));
            AABBox = AABB.GetGeometryAABB(ObjectGeometry, this);
        }

        public override void SetMass(double mass)
        {
            MassInfo.Mass = mass;
            if (MassInfo.Mass > 0.0)
                MassInfo.InverseMass = 1.0 / MassInfo.Mass;
        }

        public void SetPartialMass(double[] mass)
        {
            PartialMass = new double[mass.Length];
            Array.Copy(mass, PartialMass, mass.Length);

            for (int i = 0; i < mass.Length; i++)
                MassInfo.Mass += mass[i];

            if (IsStatic)
            {
                MassInfo.Mass = 0.0;
                MassInfo.InverseMass = 0.0;
            }
            else if (MassInfo.Mass > 0.0)
                MassInfo.InverseMass = 1.0 / MassInfo.Mass;
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
        
        private IGeometry[] GetShapesGeometry(
            List<Vector3[]> inputVertexPosition,
            List<int[][]> inputTriangle)
        {
            IGeometry[] geometry = new IGeometry[inputVertexPosition.Count];
            List<Vector3> verticesSum = new List<Vector3>();
            
            for (int i = 0; i < inputVertexPosition.Count; i++)
            {
                TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(inputTriangle[i]);

                var idx = new int[inputVertexPosition[i].Length];

                for (int j = 0; j < inputVertexPosition[i].Length; j++)
                    idx[j] = verticesSum.Count + j;
                
                geometry[i] = new Geometry(this, idx, triangleMeshes, ObjectGeometryType.ConvexShape, true);

                verticesSum.AddRange(inputVertexPosition[i].ToList());
            }

            Vertices = verticesSum.ToArray();

            SetGeometry();

            return geometry;
        }

        private void SetGeometry()
        {
            //ObjectGeometry = new Geometry(
            //    this,
            //    InputVertexPosition,
            //    ObjectGeometryType.ConvexShape);
        }

        private void SetShapesGeometry(IGeometry[] geometry)
        {
            ShapesGeometry = geometry;

            if (geometry != null)
            {
                for (int i = 0; i < ShapesGeometry.Length; i++)
                {
                    if (ShapesGeometry[i].VerticesIdx != null &&
                        ShapesGeometry[i].VerticesIdx.Length > 0)
                    {
                        for (int j = 0; j < ShapesGeometry[i].VerticesIdx.Length; j++)
                        {
                            Vertices[ShapesGeometry[i].VerticesIdx[j].ID] += StartCompoundPositionObjects[i];
                        }
                    }
                }
                CompoundingConvexObjCount = ShapesGeometry.Length;
            }

            SetShapesProperties();
            SetAABB();
        }

        private void SetShapesProperties()
        {
            Matrix3x3 baseTensors = new Matrix3x3();

            int totalVertex = 0;

            InitCenterOfMass = CalculateCenterOfMass();

            for (int i = 0; i < ShapesGeometry.Length; i++)
            {
                var vertices = ShapesGeometry[i].GetVertices();

                baseTensors += ShapeCommonUtilities.GetInertiaTensor(
                    vertices,
                    ShapesGeometry[i].Triangle,
                    InitCenterOfMass,
                    PartialMass[i]).InertiaTensor;

                Vector3[] vertexPosition = vertices;

                totalVertex += ShapesGeometry[i].VerticesIdx.Length;
            }

            RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));

            SetRelativePosition();

            MassInfo.InertiaTensor = baseTensors;
            MassInfo.InverseBaseInertiaTensor = Matrix3x3.Invert(baseTensors);
            MassInfo.InverseInertiaTensor = (RotationMatrix * MassInfo.InverseBaseInertiaTensor) *
                                            Matrix3x3.Transpose(RotationMatrix);
        }

        private void SetRelativePosition()
        {
            VerticesRelPos = new Vector3[Vertices.Length];
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

        private Vector3 CalculateCenterOfMass()
        {
            Vector3 startPosition = new Vector3();

            for (int i = 0; i < ShapesGeometry.Length; i++)
            {
                Vector3[] vertices = ShapesGeometry[i].GetVertices();

                var centerOfMass = ShapeCommonUtilities.CalculateCenterOfMass(
                    vertices,
                    ShapesGeometry[i].Triangle,
                    PartialMass[i]);
                                
                startPosition += centerOfMass * PartialMass[i];
            }

            if (MassInfo.Mass > 0.0)
                return startPosition / MassInfo.Mass;

            return Vector3.ToZero();
        }

        #endregion
    }
}
