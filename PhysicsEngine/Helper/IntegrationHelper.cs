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
using System.Collections.Concurrent;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Helper
{
    internal sealed class IntegrationHelper
    {
        #region Fields

        private readonly PhysicsEngineParameters EngineParameters;

        #endregion 

        #region Constructor

        public IntegrationHelper(PhysicsEngineParameters engineParameters)
        {
            EngineParameters = engineParameters;
        }

        #endregion Constructor

        #region Public Methods

        /// <summary>
        /// Update object velocity
        /// </summary>
        /// <param name="contact"></param>
        /// <param name="X"></param>
        public void UpdateVelocity(
            JacobianConstraint[] contact,
            double[] x)
        {
            var rangePartitioner = Partitioner.Create(0, contact.Length, Convert.ToInt32(contact.Length / EngineParameters.MaxThreadNumber) + 1);
            
            //Critical section variable
            var sync = new object();

            Parallel.ForEach(
                rangePartitioner,
                new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                (range, loopState) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        if (Math.Abs(x[i]) > 1E-50)
                        {
                            double impulse = x[i];

                            JacobianConstraint ct = contact[i];

                            if (ct.LinearComponentA.HasValue)
                                UpdateObjectVelocity(
                                    ct.ObjectA,
                                    ct.LinearComponentA.Value,
                                    ct.AngularComponentA,
                                    impulse,
                                    sync);
                            else
                                UpdateObjectVelocity(
                                    ct.ObjectA,
                                    ct.AngularComponentA,
                                    impulse,
                                    sync);

                            if (ct.LinearComponentB.HasValue)
                                UpdateObjectVelocity(
                                    ct.ObjectB,
                                    ct.LinearComponentB.Value,
                                    ct.AngularComponentB,
                                    impulse,
                                    sync);
                            else
                                UpdateObjectVelocity(
                                    ct.ObjectB,
                                    ct.AngularComponentB,
                                    impulse,
                                    sync);

                            if (ct.StartImpulse != null)
                                ct.StartImpulse.SetStartValue(impulse);
                        }
                    }
                });
        }

        /// <summary>
        /// Integrates the objects position for ConvexShape and CompoundShape.
        /// </summary>
        public void IntegrateObjectsPosition(
            ref IShape[] shapes,
            double timeStep)
        {
            
            var dynamicShapes = shapes.Where(x => !x.IsStatic);

            foreach (var shape in dynamicShapes.OfType<ISoftShape>())
                IntegrateSoftShapePosition(shape, timeStep);
            
            foreach (var shape in dynamicShapes.OfType<ConvexShape>())
                IntegrateRigidShapePosition(shape, timeStep);
            
            foreach (var shape in dynamicShapes.OfType<CompoundShape>())
                IntegrateRigidShapePosition(shape, timeStep);

        }
        
        #endregion

        #region Private Methods

        private void UpdateObjectVelocity(
            IShapeCommon simObj,
            Vector3 linearComponent,
            Vector3 angularComponent,
            double X,
            object sync)
        {
            if (!simObj.IsStatic)
            {
                Vector3 linearImpulse = X * linearComponent;
                Vector3 angularImpuse = X * angularComponent;

                Vector3 linearVelocity = linearImpulse *
                                         simObj.InverseMass;

                Vector3 angularVelocity = simObj.InertiaTensor *
                                          angularImpuse;

                //Critical Section
                lock (sync)
                {
                    simObj.SetLinearVelocity(simObj.LinearVelocity + linearVelocity);
                    simObj.SetAngularVelocity(simObj.AngularVelocity + angularVelocity);
                }
            }
        }

        private void UpdateObjectVelocity(
            IShapeCommon simObj,
            Vector3 angularComponent,
            double X,
            object sync)
        {
            if (!simObj.IsStatic)
            {
                Vector3 angularImpuse = X * angularComponent;

                Vector3 angularVelocity = simObj.InertiaTensor *
                                          angularImpuse;

                //Critical Section
                lock (sync)
                {
                    simObj.SetAngularVelocity(simObj.AngularVelocity + angularVelocity);
                }
            }
        }


        private void IntegrateRigidShapePosition(
            IShape shape,
            double timeStep)
        {
            #region Linear Velocity

            shape.SetPosition(
                    shape.Position +
                    timeStep *
                    shape.LinearVelocity);

            var externalForce = shape.ForceValue * shape.InverseMass +
                                timeStep * EngineParameters.ExternalForce;

            shape.SetLinearVelocity(shape.LinearVelocity + externalForce);

            shape.SetForce(new Vector3());

            double linearVelocity = shape.LinearVelocity.Length();

            #endregion

            #region Angular Velocity

            double angularVelocity = shape.AngularVelocity.Length();
                        
            if (angularVelocity < EngineParameters.AngularValocityMinLimit)
            {
                shape.SetAngularVelocity(shape.InertiaTensor * shape.TorqueValue);
            }
            else
            {
                Vector3 versor = shape.AngularVelocity.Normalize();

                double rotationAngle = angularVelocity * timeStep;

                var rotationQuaternion = new Quaternion(versor, rotationAngle);

                shape.SetRotationStatus((rotationQuaternion * shape.RotationStatus).Normalize());

                shape.SetRotationMatrix(shape.RotationStatus.ConvertToMatrix());

                shape.SetInertiaTensor(
                    (shape.RotationMatrix * shape.BaseInertiaTensor) *
                    shape.RotationMatrix.Transpose());

                shape.SetAngularVelocity(shape.AngularVelocity +
                                            shape.InertiaTensor *
                                            shape.TorqueValue);
            }
            

            angularVelocity = shape.AngularVelocity.Length();
            shape.SetTorque(new Vector3());

            #endregion

            #region Sleeping Object

            if (EngineParameters.SleepingObject)
                ObjectSleep(shape);

            #endregion

            #region Update AABB

            if (ShapeDefinition.Helper.GetGeometry(shape) != null &&
                (linearVelocity > 0.0 || angularVelocity > 0.0))
            {
                shape.SetAABB();
            }

            #endregion
        }

        private void IntegrateSoftShapePosition(
            ISoftShape shape,
            double timeStep)
        {
            Parallel.ForEach(shape.ShapePoints, new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    point =>
                {
                    #region Linear Velocity

                    point.SetPosition(
                            point.Position +
                            timeStep *
                            point.LinearVelocity);

                    point.SetLinearVelocity(point.LinearVelocity +
                        (point.ForceValue * point.InverseMass) +
                        (timeStep * EngineParameters.ExternalForce));

                    point.SetForce(new Vector3());

                    #endregion

                    #region Angular Velocity

                    double angularVelocity = point.AngularVelocity.Length();

                    Vector3 versor = point.AngularVelocity.Normalize();

                    double rotationAngle = angularVelocity * timeStep;

                    var rotationQuaternion = new Quaternion(versor, rotationAngle);

                    point.SetRotationStatus((rotationQuaternion * point.RotationStatus).Normalize());

                    point.SetRotationMatrix(point.RotationStatus.ConvertToMatrix());

                    point.SetInertiaTensor(
                        (point.RotationMatrix * point.BaseInertiaTensor) *
                        point.RotationMatrix.Transpose());

                    point.SetAngularVelocity(point.AngularVelocity);

                    #endregion
                });

            shape.SetAABB();
        }

        private void ObjectSleep(IShape simulationObj)
        {
            if (simulationObj.LinearVelocity.Length() <= EngineParameters.LinearVelDisable &&
                simulationObj.AngularVelocity.Length() <= EngineParameters.AngularVelDisable)
            {
                if (simulationObj.SleepingFrameCount < EngineParameters.SleepingFrameLimit)
                    simulationObj.SetSleepingFrameCount(simulationObj.SleepingFrameCount + 1);
                else if (simulationObj.SleepingFrameCount >= EngineParameters.SleepingFrameLimit)
                {
                    simulationObj.SetLinearVelocity(new Vector3());
                    simulationObj.SetAngularVelocity(new Vector3());
                }
            }
            else
                simulationObj.SetSleepingFrameCount(0);
        }


        #endregion Privare Methods
    }
}
