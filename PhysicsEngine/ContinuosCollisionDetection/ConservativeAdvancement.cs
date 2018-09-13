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
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.SolutionIntegration;

namespace SharpPhysicsEngine.ContinuosCollisionDetection
{
    internal sealed class ConservativeAdvancement : ICCDEngine
    {
        #region Fields

        private readonly INarrowPhase collisionDetectionEngine;
        private readonly IBroadPhase broadPhaseEngine;
        private readonly IntegratePosition integratePosition;

        private Vector3d[] bufPosition;
        private Quaternion[] bufRotStatus;
        private Matrix3x3[] bufRotMatrix;
        private Matrix3x3[] bufInertiaTensor;
        private AABB[] bufAABB;

        #endregion

        #region Constructor

        public ConservativeAdvancement()
        {
            var parameters = new CollisionEngineParameters();
            var physicsEngineParams = new PhysicsEngineParameters();

            parameters.SetManifoldPoints(0);

            collisionDetectionEngine = new NarrowPhase(parameters);
            broadPhaseEngine = new AABBBroadPhase(parameters);
            integratePosition = new IntegratePosition(physicsEngineParams);
        }
        
        #endregion

        #region Public Methods

        public double? GetTimeOfImpact(
            IShape shapeA,
            IShape shapeB,
            double timeStep)
        {            
            SaveBaseData(shapeA, shapeB);

            // relative linear velocity
            Vector3d rLinearVelocity = shapeB.LinearVelocity - shapeA.LinearVelocity;
            double maxAngularVelocity = shapeA.AngularVelocity.Length() * shapeA.FarthestPoint.Length() +
                                        shapeB.AngularVelocity.Length() * shapeB.FarthestPoint.Length();
                        
            double radius = 1E-4;
            double t = 0.0;
            CollisionPoint collisionPoint = GetDistance(shapeA, shapeB);

            if (collisionPoint.Intersection)
                return 0.0;
                        
            while (collisionPoint.Distance > radius &&
                   !collisionPoint.Intersection)
            {
                double nLinear = rLinearVelocity.Dot(-1.0 * collisionPoint.CollisionNormal);
                double relDist = nLinear + maxAngularVelocity;

                t += collisionPoint.Distance / relDist;

                if (t < 0.0 || t > timeStep)
                {
                    // never hit during this timestep
                    RestoreBaseData(shapeA, shapeB);
                    return null;
                }
                
                integratePosition.IntegrateObjectPosition(shapeA, t);
                integratePosition.IntegrateObjectPosition(shapeB, t);

                collisionPoint = GetDistance(shapeA, shapeB);

                if (collisionPoint == null)
                    break;
            }

            RestoreBaseData(shapeA, shapeB);

            return t;
        }

        public double? GetAABBTimeOfImpact(
            IShape shapeA,
            IShape shapeB,
            double timeStep)
        {
            SaveBaseData(shapeA, shapeB);

            // relative linear velocity
            Vector3d rLinearVelocity = shapeB.LinearVelocity - shapeA.LinearVelocity;
            double maxAngularVelocity = shapeA.AngularVelocity.Length() * shapeA.FarthestPoint.Length() +
                                        shapeB.AngularVelocity.Length() * shapeB.FarthestPoint.Length();

            double radius = 1E-4;
            double t = 0.0;
            Vector3d collisionPoint = GetAABBDist(shapeA, shapeB);
                        
            double distance = collisionPoint.Length();
                        
            while (distance > radius)
            {
                double nLinear = rLinearVelocity.Dot(collisionPoint.Normalize());
                double relDist = nLinear + maxAngularVelocity;

                t += distance / relDist;

                if (t < 0.0 || t > timeStep)
                {
                    // never hit during this timestep
                    RestoreBaseData(shapeA, shapeB);
                    return null;
                }

                integratePosition.IntegrateObjectPosition(shapeA, t);
                integratePosition.IntegrateObjectPosition(shapeB, t);

                collisionPoint = GetAABBDist(shapeA, shapeB);

                distance = collisionPoint.Length();
            }

            RestoreBaseData(shapeA, shapeB);

            return t;
        }

        #endregion

        #region Private Methods

        private CollisionPoint GetDistance(
            IShape shapeA,
            IShape shapeB)
        {
            CollisionPointStructure collisionPoint = collisionDetectionEngine.Execute(shapeA, shapeB, double.MaxValue);

            if (collisionPoint == null)
                return null;

            //TODO Gestire soft body, compound shape, concaveshape
            return collisionPoint.CollisionPointBase[0].CollisionPoint;
        }

        private Vector3d GetAABBDist(
            IShape shapeA,
            IShape shapeB)
        {
            return broadPhaseEngine.Execute(shapeA.AABBox, shapeB.AABBox);
        }

        private void SaveBaseData(
            IShape shapeA,
            IShape shapeB)
        {
            bufPosition = new Vector3d[] { shapeA.Position, shapeB.Position };
            bufRotStatus = new Quaternion[] { shapeA.RotationStatus, shapeB.RotationStatus };
            bufRotMatrix = new Matrix3x3[] { shapeA.RotationMatrix, shapeB.RotationMatrix };
            bufInertiaTensor = new Matrix3x3[] { shapeA.MassInfo.InverseInertiaTensor, shapeB.MassInfo.InverseInertiaTensor };
            bufAABB = new AABB[] { shapeA.AABBox, shapeB.AABBox };
        }

        private void RestoreBaseData(
            IShape shapeA,
            IShape shapeB)
        {
            shapeA.SetPosition(bufPosition[0]);
            shapeB.SetPosition(bufPosition[1]);

            shapeA.SetRotationStatus(bufRotStatus[0]);
            shapeB.SetRotationStatus(bufRotStatus[1]);

            shapeA.SetRotationMatrix(bufRotMatrix[0]);
            shapeB.SetRotationMatrix(bufRotMatrix[1]);

            shapeA.SetInverseInertiaTensor(bufInertiaTensor[0]);
            shapeB.SetInverseInertiaTensor(bufInertiaTensor[1]);

            shapeA.SetAABB(bufAABB[0]);
            shapeB.SetAABB(bufAABB[1]);
        }

        #endregion
    }
}
