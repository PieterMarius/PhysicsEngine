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
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class ConcaveShape : ICollisionShape, IMapper
    {
        #region Fields

        ShapeDefinition.ConcaveShape concaveShape;

        #endregion

        #region Constructor

        public ConcaveShape(
            Vector3[] inputVertexPosition,
            int[][] inputTriangle,
            Vector3 position,
            double mass,
            bool isStatic)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(inputTriangle);

            IConvexHullEngine convexHullEngine = new ConvexHullEngine();

            this.concaveShape = new ShapeDefinition.ConcaveShape(triangleMeshes, inputVertexPosition, convexHullEngine, position, mass, isStatic);
        }

        #endregion

        #region Public Methods

        IShape IMapper.GetShape()
        {
            return concaveShape;
        }

        public ObjectType ObjectType
        {
            get
            {
                return concaveShape.ObjectType;
            }
        }

        public Vector3 Position
        {
            get
            {
                return concaveShape.Position;
            }
        }

        public Vector3 InitCenterOfMass
        {
            get
            {
                return concaveShape.InitCenterOfMass;
            }
        }

        public Vector3 LinearVelocity
        {
            get
            {
                return concaveShape.LinearVelocity;
            }
        }

        public Vector3 AngularVelocity
        {
            get
            {
                return concaveShape.AngularVelocity;
            }
        }

        public Matrix3x3 InertiaTensor
        {
            get
            {
                return concaveShape.MassInfo.InverseInertiaTensor;
            }
        }

        public Matrix3x3 BaseInertiaTensor
        {
            get
            {
                return concaveShape.MassInfo.InverseBaseInertiaTensor;
            }
        }

        public Matrix3x3 RotationMatrix
        {
            get
            {
                return concaveShape.RotationMatrix;
            }
        }

        public double Mass
        {
            get
            {
                return concaveShape.MassInfo.Mass;
            }
        }

        public double InverseMass
        {
            get
            {
                return concaveShape.MassInfo.InverseMass;
            }
        }

        public Vector3 ForceValue
        {
            get
            {
                return concaveShape.ForceValue;
            }
        }

        public bool IsStatic
        {
            get
            {
                return concaveShape.IsStatic;
            }
        }

        public void ExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
        {
            concaveShape.SetExcludeFromCollisionDetection(excludeFromCollisionDetection);
        }

        public Vector3 GetCenterOfMassShiftValue(int index = 0)
        {
            return -1.0 * concaveShape.InitCenterOfMass;
        }

        public int GetGeometryCount()
        {
            return 1;
        }

        public int GetID()
        {
            return concaveShape.ID;
        }

        public Vector3 GetMaxAABB()
        {
            return CommonUtilities.GetAABBMaxValue(concaveShape.ConvexShapesGeometry);
        }

        public Vector3 GetMinAABB()
        {
            return CommonUtilities.GetAABBMinValue(concaveShape.ConvexShapesGeometry);
        }

        public Vector3[] GetVertices()
        {
            return concaveShape.InputVertexPosition;
        }

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            concaveShape.SetAngularVelocity(inputAngularVelocity);
        }

        public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
        {
            concaveShape.SetInverseBaseInertiaTensor(inputIntertiaTensor);
        }

        public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
        {
            concaveShape.SetDynamicFrictionCoeff(dynamicFrictionCoeff);
        }

        public void SetErrorReductionParam(double value)
        {
            concaveShape.SetRestoreCoeff(value);
        }

        public void SetForce(Vector3 force)
        {
            concaveShape.SetForce(force);
        }

        public void SetInverseInertiaTensor(Matrix3x3 inertiaTensor)
        {
            concaveShape.SetInverseInertiaTensor(inertiaTensor);
        }

        public void SetIsStatic(bool isStatic)
        {
            concaveShape.SetIsStatic(isStatic);
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            concaveShape.SetLinearVelocity(inputLinearVelocity);
        }

        public void SetMass(double mass)
        {
            concaveShape.SetMass(mass);
        }

        public void SetPosition(Vector3 inputPosition)
        {
            concaveShape.SetPosition(inputPosition);
        }

        public void SetRestitutionCoeff(double restitutionCoeff)
        {
            concaveShape.SetRestitutionCoeff(restitutionCoeff);
        }

        public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
        {
            concaveShape.SetRotationMatrix(inputRotationMatrix);
        }

        public void SetRotationStatus(Quaternion inputRotationStatus)
        {
            concaveShape.SetRotationStatus(inputRotationStatus);
        }

        public void SetSleepingFrameCount(int frameCount)
        {
            concaveShape.SetSleepingFrameCount(frameCount);
        }

        public void SetStaticFrictionCoeff(double staticFrictionCoeff)
        {
            concaveShape.SetStaticFrictionCoeff(staticFrictionCoeff);
        }

        public void SetTorque(Vector3 torque)
        {
            concaveShape.SetTorque(torque);
        }

        public double[][][] GetConvexShapeList()
        {
            Vector3[][] result = new Vector3[concaveShape.ConvexShapesGeometry.Length][];

            for (int i = 0; i < concaveShape.ConvexShapesGeometry.Length; i++)
                result[i] = Array.ConvertAll(concaveShape.ConvexShapesGeometry[i].RelativePosition, 
                                             x => ShapeDefinition.Helper.GetVertexPosition(concaveShape.ConvexShapesGeometry[i], x));

            return GeneralMathUtilities.GetMatrixFromVector3Matrix(result);
        }

        #endregion
    }
}
