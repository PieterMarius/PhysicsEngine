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
            int ID_B)
        {
            GJKOutput gjkOutput = collisionEngine.Execute(objA, objB);
                        
            return NarrowPhaseCollisionDetection(
                gjkOutput,
                objA,
                objB,
                ID_A,
                ID_B);
        }

        #endregion

        #region Private Methods

        private CollisionPointStructure NarrowPhaseCollisionDetection(
            GJKOutput gjkOutput,
            VertexProperties[] vertexObjA,
            VertexProperties[] vertexObjB,
            int ID_A,
            int ID_B)
        {
            if (gjkOutput.Intersection)
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
            else if (gjkOutput.CollisionDistance <= parameters.CollisionDistance)
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

        #endregion

    }
}
