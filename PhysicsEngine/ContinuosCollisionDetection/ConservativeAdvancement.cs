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
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.ContinuosCollisionDetection
{
    internal sealed class ConservativeAdvancement : ICCDEngine
    {
        #region Fields

        private readonly INarrowPhase collisionDetectionEngine;
        private readonly IntegratePosition integratePosition;

        private Vector3[] bufPosition;
        private Quaternion[] bufRotStatus;
        private Matrix3x3[] bufRotMatrix;
        private Matrix3x3[] bufInertiaTensor;

        #endregion

        #region Constructor

        public ConservativeAdvancement()
        {
            var parameters = new CollisionEngineParameters();
            var physicsEngineParams = new PhysicsEngineParameters();

            parameters.SetManifoldPoints(0);

            collisionDetectionEngine = new NarrowPhase(parameters);
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
            Vector3 rLinearVelocity = shapeB.LinearVelocity - shapeA.LinearVelocity;
            double maxAngularVelocity = shapeA.AngularVelocity.Length() * shapeA.FarthestPoint.Length() +
                                        shapeB.AngularVelocity.Length() * shapeB.FarthestPoint.Length();

            //TEST
            var a = 0.0;
            if (maxAngularVelocity > 10.0)
                a = 0.0;

            double radius = 1E-4;
            double t = 0.0;
            CollisionPoint collisionPoint = GetDistance(shapeA, shapeB);

            if (collisionPoint == null || collisionPoint.Intersection)
                return 0.0;
                        
            while (collisionPoint.Distance > radius &&
                   !collisionPoint.Intersection)
            {
                double nLinear = rLinearVelocity.Dot(-1.0 * collisionPoint.CollisionNormal);
                double relDist = nLinear + maxAngularVelocity;

                t += collisionPoint.Distance / relDist;

                if (t < 0.0 || t > 1.0)
                {
                    // never hit
                    RestoreBaseData(shapeA, shapeB);
                    return null;
                }
                
                integratePosition.IntegrateObjectPosition(shapeA, t);
                integratePosition.IntegrateObjectPosition(shapeB, t);

                collisionPoint = GetDistance(shapeA, shapeB);

                if (collisionPoint == null || 
                    collisionPoint.Intersection ||
                    collisionPoint.Distance < 1E-10)
                {
                    break;
                }
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
            var collisionPoint = collisionDetectionEngine.Execute(shapeA, shapeB, 100.0);

            if (collisionPoint == null)
                return null;

            //Gestire soft body, compound shape, concaveshape
            return collisionPoint.CollisionPointBase[0].CollisionPoint;
        }

        private void SaveBaseData(
            IShape shapeA,
            IShape shapeB)
        {
            bufPosition = new Vector3[] { shapeA.Position, shapeB.Position };
            bufRotStatus = new Quaternion[] { shapeA.RotationStatus, shapeB.RotationStatus };
            bufRotMatrix = new Matrix3x3[] { shapeA.RotationMatrix, shapeB.RotationMatrix };
            bufInertiaTensor = new Matrix3x3[] { shapeA.InertiaTensor, shapeB.InertiaTensor };
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

            shapeA.SetInertiaTensor(bufInertiaTensor[0]);
            shapeB.SetInertiaTensor(bufInertiaTensor[1]);
        }

        #endregion
    }
}
