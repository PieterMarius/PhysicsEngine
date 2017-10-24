using SharpEngineMathUtility;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Helper
{
    public sealed class IntegrationHelper
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
            SolutionValues[] X)
        {
            //Critical section variable
            var sync = new object();

            Parallel.For(0, contact.Length, new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                i =>
                {
                    if (Math.Abs(X[i].X) > 1E-50)
                    {
                        double impulse = X[i].X;

                        JacobianConstraint ct = contact[i];

                        UpdateObjectVelocity(
                            ct.ObjectA,
                            ct.LinearComponentA,
                            ct.AngularComponentA,
                            impulse,
                            sync);

                        UpdateObjectVelocity(
                            ct.ObjectB,
                            ct.LinearComponentB,
                            ct.AngularComponentB,
                            impulse,
                            sync);

                        ct.StartImpulse.SetStartValue(impulse * EngineParameters.WarmStartingValue);
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
            foreach (var shape in shapes)
            {
                if (shape.ObjectType != ObjectType.StaticBody)
                {
                    ISoftShape softShape = shape as ISoftShape;

                    if (softShape == null)
                        IntegrateRigidShapePosition(shape, timeStep);
                    else
                        IntegrateSoftShapePosition(softShape, timeStep);
                }
            }
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
            if (simObj.ObjectType != ObjectType.StaticBody)
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
                    //TODO implementare x Soft Body
                    simObj.SetLinearVelocity(simObj.LinearVelocity + linearVelocity);
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

            shape.SetLinearVelocity(shape.LinearVelocity +
                (shape.ForceValue * shape.InverseMass) +
                (timeStep * EngineParameters.ExternalForce));

            shape.SetForce(new Vector3());

            double linearVelocity = shape.LinearVelocity.Length();

            #endregion

            #region Angular Velocity

            double angularVelocity = shape.AngularVelocity.Length();

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
            foreach (var point in shape.ShapePoints)
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

                double linearVelocity = point.LinearVelocity.Length();

                #endregion
            }

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
