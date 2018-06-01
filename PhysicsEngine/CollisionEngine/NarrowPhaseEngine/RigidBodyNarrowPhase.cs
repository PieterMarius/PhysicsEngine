using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class RigidBodyNarrowPhase
    {
        #region Fields

        private const double normalTolerance = 1E-15;

        private GJK collisionEngine;
        private EPA compenetrationEngine;
        private readonly CollisionEngineParameters parameters;
        private readonly CollisionEngineParameters collisionEngineParameters;
        private readonly ManifoldPointsGenerator manifoldGJKPointsGenerator;
        private readonly ManifoldPointsGenerator manifoldEPAPointsGenerator;
        private readonly AABBBroadPhase innerBroadPhase;

        #endregion

        #region Constructor

        public RigidBodyNarrowPhase(CollisionEngineParameters parameters)
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

        public List<CollisionPointStructure> RigidBodyCollisionStep(
            IShape A,
            IShape B)
        {
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            IGeometry[] geometryA = ShapeDefinition.Helper.GetGeometry(A);
            IGeometry[] geometryB = ShapeDefinition.Helper.GetGeometry(B);

            List<CollisionPair> collisionPair = CheckGeometryAABB(
                geometryA,
                geometryB);

            foreach (var collidingPair in collisionPair)
            {
                VertexProperties[] vertexObjA = Helper.SetVertexPosition(geometryA[collidingPair.objectIndexA]);
                VertexProperties[] vertexObjB = Helper.SetVertexPosition(geometryB[collidingPair.objectIndexB]);

                GJKOutput gjkOutput = collisionEngine.Execute(vertexObjA, vertexObjB);

                CollisionPointStructure collision = NarrowPhaseCollisionDetection(
                    gjkOutput,
                    vertexObjA,
                    vertexObjB,
                    A.ID,
                    B.ID);

                if (collision != null)
                    collisionPointStructure.Add(collision);
            }

            return collisionPointStructure;
        }

        private List<CollisionPair> CheckGeometryAABB(
            IGeometry[] geometryA,
            IGeometry[] geometryB)
        {
            var geometryBoxesA = Array.ConvertAll(geometryA, x => x.AABBox);
            var geometryBoxesB = Array.ConvertAll(geometryB, x => x.AABBox);

            if (geometryBoxesA.Length == 1 &&
                geometryBoxesB.Length == 1)
                return new List<CollisionPair>() { new CollisionPair(0, 0) };

            return innerBroadPhase.Execute(geometryBoxesA, geometryBoxesB, parameters.CollisionDistance);
        }

        private CollisionPointStructure NarrowPhaseCollisionDetection(
            GJKOutput gjkOutput,
            VertexProperties[] vertexObjA,
            VertexProperties[] vertexObjB,
            int ID_A,
            int ID_B)
        {
            if (!gjkOutput.Intersection &&
                gjkOutput.CollisionDistance <= parameters.CollisionDistance)
            {
                if (gjkOutput.CollisionNormal.Length() < normalTolerance)
                    return null;

                List<CollisionPoint> collisionPointsList = manifoldGJKPointsGenerator.GetManifoldPoints(
                    Array.ConvertAll(vertexObjA, x => x.Vertex),
                    Array.ConvertAll(vertexObjB, x => x.Vertex),
                    gjkOutput.CollisionPoint);

                var collisionPointBaseStr = new CollisionPointBaseStructure(
                        gjkOutput.CollisionPoint,
                        collisionPointsList.ToArray());

                return new CollisionPointStructure(
                    ID_A,
                    ID_B,
                    collisionPointBaseStr);
            }
            else if (gjkOutput.Intersection)
            {
                EPAOutput epaOutput = compenetrationEngine.Execute(
                                                vertexObjA,
                                                vertexObjB,
                                                gjkOutput.SupportTriangles,
                                                gjkOutput.Centroid);

                if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
                    return null;

                List<CollisionPoint> collisionPointsList = manifoldEPAPointsGenerator.GetManifoldPoints(
                                                               Array.ConvertAll(vertexObjA, x => x.Vertex),
                                                               Array.ConvertAll(vertexObjB, x => x.Vertex),
                                                               epaOutput.CollisionPoint);

                var collisionPointBaseStr = new CollisionPointBaseStructure(
                        epaOutput.CollisionPoint,
                        collisionPointsList.ToArray());

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
