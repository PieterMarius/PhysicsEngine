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

using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.SolutionIntegration
{
    internal sealed class IntegratePosition
    {
        #region Fields

        private readonly PhysicsEngineParameters EngineParameters;

        #endregion 

        #region Constructor

        public IntegratePosition(PhysicsEngineParameters engineParameters)
        {
            EngineParameters = engineParameters;
        }

        #endregion Constructor

        #region Public Methods

        /// <summary>
        /// Integrates the objects position for ConvexShape and CompoundShape.
        /// </summary>
        public void IntegrateObjectsPosition(
            ref IShape[] shapes,
            double timeStep)
        {
            var dynamicShapes = shapes.Where(x => !x.IsStatic).ToList();

            foreach (var shape in dynamicShapes.OfType<ISoftShape>())
                IntegrateSoftShapePosition(shape, timeStep, true);

            foreach (var shape in dynamicShapes.OfType<ConvexShape>())
            {
                IntegrateRigidShapePosition(shape, timeStep);
                IntegrateExternalForce(shape, timeStep);
                UpdateShapeProperties(shape);
            }

            foreach (var shape in dynamicShapes.OfType<CompoundShape>())
            {
                IntegrateRigidShapePosition(shape, timeStep);
                IntegrateExternalForce(shape, timeStep);
                UpdateShapeProperties(shape);
            }

            foreach (var shape in dynamicShapes.OfType<ConcaveShape>())
            {
                IntegrateRigidShapePosition(shape, timeStep);
                IntegrateExternalForce(shape, timeStep);
                UpdateShapeProperties(shape);
            }
        }


        /// <summary>
        /// Update Position, Rotation Status, Rotation Matrix, Inertia Tensor
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="timeStep"></param>
        public void IntegrateObjectPosition(
            IShape shape,
            double timeStep)
        {
            if(shape is ISoftShape)
                IntegrateSoftShapePosition((ISoftShape)shape, timeStep, false);
            else 
                IntegrateRigidShapePosition(shape, timeStep);

            shape.SetAABB();
        }

        #endregion

        #region Private Methods

        private void UpdateLinearVelocity(
            IShape shape,
            double timeStep)
        {
            var externalForce = shape.ForceValue * shape.MassInfo.InverseMass +
                                timeStep * EngineParameters.ExternalForce;

            shape.SetLinearVelocity(shape.LinearVelocity + externalForce);
        }

        private void UpdateAngularVelocity(
            IShape shape,
            double timeStep)
        {
            var extAngularVelocity = shape.MassInfo.InertiaTensor * shape.TorqueValue;

            shape.SetAngularVelocity(shape.AngularVelocity + extAngularVelocity);
        }

        private void IntegrateExternalForce(
            IShape shape,
            double timeStep)
        {
            UpdateLinearVelocity(shape, timeStep);
            shape.SetForce(new Vector3d());

            UpdateAngularVelocity(shape, timeStep);
            shape.SetTorque(new Vector3d());
        }

        private void UpdateShapeProperties(IShape shape)
        {
            #region Update AABB

            double linearVelocity = shape.LinearVelocity.Length();
            double angularVelocity = shape.AngularVelocity.Length();
            shape.AABBox.SetPositionChanged(false);

            if (ShapeDefinition.Helper.GetGeometry(shape) != null &&
                (linearVelocity > 0.0 || angularVelocity > 0.0))
            {
                shape.SetAABB();
                shape.AABBox.SetPositionChanged(true);
            }

            #endregion

            #region Sleeping Object

            if (EngineParameters.SleepingObject)
                ObjectSleep(shape);

            #endregion
        }

        private void IntegrateRigidShapePosition(
            IShape shape,
            double timeStep)
        {
            if (EngineParameters.SleepingObject)
                ObjectSleep(shape);

            #region Linear Velocity

            shape.SetPosition(
                    shape.Position +
                    timeStep *
                    shape.LinearVelocity);
            
            #endregion

            #region Angular Velocity

            double angularVelocity = shape.AngularVelocity.Length();
            
            if (angularVelocity >= EngineParameters.AngularVelocityMinLimit)
            {
                Vector3d versor = shape.AngularVelocity.Normalize();

                double rotationAngle = angularVelocity * timeStep;

                var rotationQuaternion = new Quaternion(versor, rotationAngle);

                shape.SetRotationStatus((rotationQuaternion * shape.RotationStatus).Normalize());

                shape.SetRotationMatrix(shape.RotationStatus.ConvertToMatrix());

                shape.SetInverseInertiaTensor(
                    (shape.RotationMatrix * shape.MassInfo.InverseBaseInertiaTensor) *
                    shape.RotationMatrix.Transpose());
            }

            #endregion
        }

        private void UpdatePointLinearVelocity(
            SoftShapePoint point,
            double timeStep)
        {
            var externalForce = point.ForceValue * point.MassInfo.InverseMass +
                                timeStep * EngineParameters.ExternalForce;

            point.SetLinearVelocity(point.LinearVelocity + externalForce);
        }

        private void IntegrateSoftShapePosition(
            ISoftShape shape,
            double timeStep,
            bool updateExtForce)
        {
            Parallel.ForEach(
                shape.ShapePoints, 
                new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                point => {
                        
                    #region Linear Velocity

                    point.SetPosition(
                            point.Position +
                            timeStep *
                            point.LinearVelocity);

                    if (updateExtForce)
                    {
                        UpdatePointLinearVelocity(point, timeStep);
                        point.SetForce(new Vector3d());
                    }

                    #endregion

                    #region Angular Velocity

                    double angularVelocity = point.AngularVelocity.Length();

                    Vector3d versor = point.AngularVelocity.Normalize();

                    double rotationAngle = angularVelocity * timeStep;

                    var rotationQuaternion = new Quaternion(versor, rotationAngle);

                    point.SetRotationStatus((rotationQuaternion * point.RotationStatus).Normalize());

                    point.SetRotationMatrix(point.RotationStatus.ConvertToMatrix());

                    point.SetInverseInertiaTensor(
                        (point.RotationMatrix * point.MassInfo.InverseBaseInertiaTensor) *
                        point.RotationMatrix.Transpose());
                                                
                    #endregion
                    }
                );

            shape.SetAABB();
        }

        private void ObjectSleep(IShape simulationObj)
        {
            if (simulationObj.LinearVelocity.Length() <= EngineParameters.LinearVelDisable &&
                simulationObj.AngularVelocity.Length() <= EngineParameters.AngularVelDisable)
            {
                if (simulationObj.SleepingFrameCount < EngineParameters.SleepingFrameLimit)
                    simulationObj.SetSleepingFrameCount(simulationObj.SleepingFrameCount + 1);
                else
                {
                    simulationObj.SetLinearVelocity(new Vector3d());
                    simulationObj.SetAngularVelocity(new Vector3d());
                    simulationObj.SetIsSleeping(true);
                }
            }
            else
            {
                simulationObj.SetSleepingFrameCount(0);
                simulationObj.SetIsSleeping(false);
            }
        }

        #endregion
    }
}
