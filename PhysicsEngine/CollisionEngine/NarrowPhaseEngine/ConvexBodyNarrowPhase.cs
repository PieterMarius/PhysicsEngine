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

using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class ConvexBodyNarrowPhase
    {
        #region Fields

        private const double normalTolerance = 1E-15;

        private GJK collisionEngine;
        private EPA compenetrationEngine;
        private readonly CollisionEngineParameters parameters;
        private readonly ManifoldPointsGenerator manifoldGJKPointsGenerator;
        private readonly ManifoldPointsGenerator manifoldEPAPointsGenerator;
        private readonly AABBBroadPhase innerBroadPhase;

        #endregion

        #region Constructor

        public ConvexBodyNarrowPhase(CollisionEngineParameters parameters)
        {
            this.parameters = parameters;

            collisionEngine = new GJK(
                parameters.MaxGJKIteration,
                parameters.Precision,
                parameters.GJKManifoldTolerance,
                parameters.ManifoldPointNumber);

            compenetrationEngine = new EPA(
                parameters.MaxEPAIteration,
                parameters.Precision,
                parameters.EPAManifoldTolerance,
                parameters.ManifoldPointNumber);

            manifoldGJKPointsGenerator = new ManifoldPointsGenerator(
                parameters.ManifoldPointNumber,
                parameters.GJKManifoldTolerance,
                parameters.ManifoldProjectionTolerance);

            manifoldEPAPointsGenerator = new ManifoldPointsGenerator(
                parameters.ManifoldPointNumber,
                parameters.EPAManifoldTolerance,
                parameters.ManifoldProjectionTolerance);

            innerBroadPhase = new AABBBroadPhase(parameters);
        }

        #endregion

        #region Public Methods

        public CollisionPointStructure Execute(
            VertexProperties[] objA,
            VertexProperties[] objB,
            int ID_A,
            int ID_B,
            double collisionDistance)
        {
            GJKOutput gjkOutput = collisionEngine.Execute(objA, objB);
                        
            return NarrowPhaseCollisionDetection(
                gjkOutput,
                objA,
                objB,
                ID_A,
                ID_B,
                collisionDistance);
        }

        #endregion

        #region Private Methods

        private CollisionPointStructure NarrowPhaseCollisionDetection(
            GJKOutput gjkOutput,
            VertexProperties[] vertexObjA,
            VertexProperties[] vertexObjB,
            int ID_A,
            int ID_B,
            double collisionDistance)
        {
            if (gjkOutput.Intersection)
            {
                return ExecuteEPAEngine(
                    gjkOutput,
                    vertexObjA,
                    vertexObjB,
                    ID_A,
                    ID_B);
            }
            else if (gjkOutput.CollisionDistance <= collisionDistance && 
                     gjkOutput.CollisionDistance >= 0.0)
            {
                if (gjkOutput.CollisionNormal.Length() < normalTolerance)
                    return null;

                List<CollisionPoint> collisionPointsList = null;

                collisionPointsList = manifoldGJKPointsGenerator.GetManifoldPoints(
                    Array.ConvertAll(vertexObjA, x => x.Vertex),
                    Array.ConvertAll(vertexObjB, x => x.Vertex),
                    gjkOutput.CollisionPoint);

                gjkOutput.CollisionPoint.SetDistance(gjkOutput.CollisionDistance);

                var collisionPointBaseStr = new CollisionPointBaseStructure(
                        gjkOutput.CollisionPoint,
                        collisionPointsList?.ToArray());

                return new CollisionPointStructure(
                    ID_A,
                    ID_B,
                    collisionPointBaseStr);
            }

            return null;
        }

        private CollisionPointStructure ExecuteEPAEngine(
            GJKOutput gjkOutput,
            VertexProperties[] vertexObjA,
            VertexProperties[] vertexObjB,
            int ID_A,
            int ID_B)
        {
            EPAOutput epaOutput = compenetrationEngine.Execute(
                                                vertexObjA,
                                                vertexObjB,
                                                gjkOutput.SupportTriangles,
                                                gjkOutput.Centroid);

            if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
                return null;
                        
            List<CollisionPoint> collisionPointsList = null;

            collisionPointsList = manifoldEPAPointsGenerator.GetManifoldPoints(
                                        Array.ConvertAll(vertexObjA, x => x.Vertex),
                                        Array.ConvertAll(vertexObjB, x => x.Vertex),
                                        epaOutput.CollisionPoint);

            var collisionPointBaseStr = new CollisionPointBaseStructure(
                    epaOutput.CollisionPoint,
                    collisionPointsList?.ToArray());

            return new CollisionPointStructure(
                ID_A,
                ID_B,
                collisionPointBaseStr);
        }

        #endregion

    }
}
