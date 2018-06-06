using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Helper
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
            var dynamicShapes = shapes.Where(x => !x.IsStatic);

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
        }

        #endregion

        #region Private Methods

        private void UpdateLinearVelocity(
            IShape shape,
            double timeStep)
        {
            var externalForce = shape.ForceValue * shape.InverseMass +
                                timeStep * EngineParameters.ExternalForce;

            shape.SetLinearVelocity(shape.LinearVelocity + externalForce);
        }

        private void UpdateAngularVelocity(
            IShape shape,
            double timeStep)
        {
            var extAngularVelocity = shape.InertiaTensor * shape.TorqueValue;

            shape.SetAngularVelocity(shape.AngularVelocity + extAngularVelocity);
        }

        private void IntegrateExternalForce(
            IShape shape,
            double timeStep)
        {
            UpdateLinearVelocity(shape, timeStep);
            shape.SetForce(new Vector3());

            UpdateAngularVelocity(shape, timeStep);
            shape.SetTorque(new Vector3());
        }

        private void UpdateShapeProperties(IShape shape)
        {
            #region Sleeping Object

            if (EngineParameters.SleepingObject)
                ObjectSleep(shape);

            #endregion

            #region Update AABB

            double linearVelocity = shape.LinearVelocity.Length();

            double angularVelocity = shape.AngularVelocity.Length();

            if (ShapeDefinition.Helper.GetGeometry(shape) != null &&
                (linearVelocity > 0.0 || angularVelocity > 0.0))
            {
                shape.SetAABB();
            }

            #endregion
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
            
            #endregion

            #region Angular Velocity

            double angularVelocity = shape.AngularVelocity.Length();
            
            if (angularVelocity >= EngineParameters.AngularVelocityMinLimit)
            {
                Vector3 versor = shape.AngularVelocity.Normalize();

                double rotationAngle = angularVelocity * timeStep;

                var rotationQuaternion = new Quaternion(versor, rotationAngle);

                shape.SetRotationStatus((rotationQuaternion * shape.RotationStatus).Normalize());

                shape.SetRotationMatrix(shape.RotationStatus.ConvertToMatrix());

                shape.SetInertiaTensor(
                    (shape.RotationMatrix * shape.BaseInertiaTensor) *
                    shape.RotationMatrix.Transpose());
            }
                                  
            #endregion
        }

        private void UpdatePointLinearVelocity(
            SoftShapePoint point,
            double timeStep)
        {
            var externalForce = point.ForceValue * point.InverseMass +
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
                        point.SetForce(new Vector3());
                    }

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
                else if (simulationObj.SleepingFrameCount >= EngineParameters.SleepingFrameLimit)
                {
                    simulationObj.SetLinearVelocity(new Vector3());
                    simulationObj.SetAngularVelocity(new Vector3());
                }
            }
            else
                simulationObj.SetSleepingFrameCount(0);
        }

        #endregion
    }
}
