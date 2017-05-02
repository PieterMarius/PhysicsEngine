﻿using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using ShapeDefinition;
using CollisionEngine.SoftBody;
using System.Linq;

namespace CollisionEngine
{
	public class CollisionDetectionEngine: ICollisionEngine
	{
        #region Private Fields

        private const double normalTolerance = 1E-15;
        
        private GJK collisionEngine;
		private EPA compenetrationCollisionEngine;
		private readonly CollisionEngineParameters collisionEngineParameters;
		private readonly SweepAndPruneEngine sweepAndPruneEngine;
        private readonly SoftBodyCollisionDetection softBodyCollisionDetection;

        private double CollisionDistance;

		#endregion

		#region Constructor

		public CollisionDetectionEngine (
			CollisionEngineParameters collisionEngineParameters,
            double collisionDistance)
		{
			this.collisionEngineParameters = collisionEngineParameters;

			collisionEngine = new GJK (
				collisionEngineParameters.MaxGJKIteration,
				collisionEngineParameters.Precision,
				collisionEngineParameters.GJKManifoldTolerance,
				collisionEngineParameters.ManifoldPointNumber);

			compenetrationCollisionEngine = new EPA (
				collisionEngineParameters.MaxEPAIteration,
				collisionEngineParameters.Precision,
				collisionEngineParameters.EPAManifoldTolerance,
				collisionEngineParameters.ManifoldPointNumber);

			sweepAndPruneEngine = new SweepAndPruneEngine (collisionEngineParameters);
            softBodyCollisionDetection = new SoftBodyCollisionDetection();

            CollisionDistance = collisionDistance;
		}

        #endregion

        #region Public Methods

        #region Interface ICollisionEngine

        /// <summary>
        /// Runs the test collision.
        /// </summary>
        /// <returns>The test collision.</returns>
        /// <param name="shapes">Objects.</param>
        /// <param name="minDistance">Minimum distance.</param>
        public List<CollisionPointStructure> Execute(IShape[] shapes)
        {
            return (collisionEngineParameters.ActivateSweepAndPrune) ?
                    SweepAndPruneBroadPhase(shapes) :
                    BruteForceBroadPhase(shapes);
        }

        public void SetCollisionDistance(double collisionDistance)
        {
            CollisionDistance = collisionDistance;
        }

		/// <summary>
		/// Gets the engine parameters.
		/// </summary>
		/// <returns>The engine parameters.</returns>
		public CollisionEngineParameters GetEngineParameters()
		{
			return collisionEngineParameters;
		}

		#endregion

		#endregion

		#region Private Methods

        private CollisionPointStructure NarrowPhaseCollisionDetection(
            GJKOutput gjkOutput,
            IGeometry A,
            IGeometry B,
            int indexA,
            int indexB)
        {
            
            if (!gjkOutput.Intersection &&
                gjkOutput.CollisionDistance <= CollisionDistance)
            {
                var mpg = new ManifoldPointsGenerator(
                                                collisionEngineParameters.ManifoldPointNumber,
                                                collisionEngineParameters.GJKManifoldTolerance,
                                                collisionEngineParameters.ManifoldProjectionTolerance);

                if (gjkOutput.CollisionNormal.Length() < normalTolerance)
                    return null;

                List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
                                                                A,
                                                                B,
                                                                gjkOutput.CollisionPoint);

                return new CollisionPointStructure(
                    indexA,
                    indexB,
                    new CollisionPointBaseStructure(
                        gjkOutput.CollisionDistance, 
                        gjkOutput.Intersection, 
                        gjkOutput.CollisionPoint,
                        collisionPointsList.ToArray()));
            }

            if (gjkOutput.Intersection)
            {
                EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
                                                A,
                                                B,
                                                gjkOutput.SupportTriangles,
                                                gjkOutput.Centroid);

                if (epaOutput.CollisionPoint.CollisionNormal.Length() < normalTolerance)
                    return null;

                var mpg = new ManifoldPointsGenerator(
                                                  collisionEngineParameters.ManifoldPointNumber,
                                                  collisionEngineParameters.EPAManifoldTolerance,
                                                  collisionEngineParameters.ManifoldProjectionTolerance);

                List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
                                                               A,
                                                               B,
                                                               epaOutput.CollisionPoint);

                return new CollisionPointStructure(
                    indexA,
                    indexB,
                    new CollisionPointBaseStructure(
                        epaOutput.CompenetrationDistance,
                        gjkOutput.Intersection,
                        epaOutput.CollisionPoint,
                        collisionPointsList.ToArray()));
            }

            return null;
        }

		private CollisionPointStructure NarrowPhase(
			IShape A,
			IShape B,
			int indexA,
			int indexB)
		{
            List<CollisionPointStructure> collisionPointStructure = GetCollisionPointStructure(
                A, 
                B, 
                indexA, 
                indexB);
            
            if (collisionPointStructure.Count > 1)
            {
                List<CollisionPointBaseStructure> baseStructure = new List<CollisionPointBaseStructure>();

                foreach (CollisionPointStructure cps in collisionPointStructure)
                {
                    if(cps != null)
                        baseStructure.AddRange(cps.CollisionPointBase);
                }

                collisionPointStructure[0].SetBaseCollisionPoint(baseStructure.ToArray());
            }

            if (collisionPointStructure.Count > 0)
                return collisionPointStructure[0];

            return null;
		}

		private List<CollisionPointStructure> BruteForceBroadPhase(IShape[] shapes)
		{
			var result = new List<CollisionPointStructure> ();

			var lockMe = new object();

            Parallel.For(0,
                shapes.Length,
                new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
                i =>
                {
                    if (shapes[i] != null)
                    {
                        for (int j = i + 1; j < shapes.Length; j++)
                        {
                            if (shapes[j] != null)
                            {
                                CollisionPointStructure collisionPointStruct = NarrowPhase(
                                                                              shapes[i],
                                                                              shapes[j],
                                                                              i,
                                                                              j);

                                lock (lockMe)
                                {
                                    if (collisionPointStruct != null)
                                        result.Add(collisionPointStruct);
                                }
                            }
                        }
                    }
                });
			
			return result;
		}

		private List<CollisionPointStructure> SweepAndPruneBroadPhase(IShape[] shapes)
		{
			var result = new List<CollisionPointStructure> ();

            AABB[][] boxs = GetAABB(shapes);
            
            List<CollisionPair> collisionPair = sweepAndPruneEngine.Execute (boxs, CollisionDistance);

            	var lockMe = new object();

            Parallel.ForEach(
                collisionPair,
                new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber },
                pair =>
                {
                    CollisionPointStructure collisionPointStruct = NarrowPhase(
                        shapes[pair.objectIndexA],
                        shapes[pair.objectIndexB],
                        pair.objectIndexA,
                        pair.objectIndexB);

                    if (collisionPointStruct != null)
                    {
                        lock (lockMe)
                        {
                            result.Add(collisionPointStruct);
                        }
                    }
                });

            return result;
		}

        private AABB[][] GetAABB(IShape[] shapes)
        {
            AABB[][] boxs = new AABB[shapes.Length][];

            foreach (var item in shapes.Select((shape, i) => new { shape, i }))
            {
                if (item.shape is IConvexShape)
                {
                    boxs[item.i] = new AABB[1];
                    boxs[item.i][0] = ((IConvexShape)item.shape).ObjectGeometry.AABBox;
                }
                else if (item.shape is ICompoundShape)
                {
                    AABB[] bufBox = Array.ConvertAll(((ICompoundShape)item.shape).ObjectGeometry, x => x.AABBox);
                    boxs[item.i] = bufBox;
                }
                else if (item.shape is ISoftShape)
                {
                    boxs[item.i] = new AABB[1];
                    boxs[item.i][0] = ((ISoftShape)item.shape).AABBox;
                }
            }

            return boxs;
        }

        private List<CollisionPointStructure> GetCollisionPointStructure(
            IShape A,
            IShape B,
            int indexA,
            int indexB)
        {
            List<CollisionPointStructure> collisionPointStructure = new List<CollisionPointStructure>();

            if (!(A is ISoftShape) &&
                !(B is ISoftShape))
            {
                IGeometry[] geometryA = ShapeDefinition.Helper.GetGeometry(A);
                IGeometry[] geometryB = ShapeDefinition.Helper.GetGeometry(B);

                for (int geometryIndexA = 0; geometryIndexA < geometryA.Length; geometryIndexA++)
                {
                    for (int geometryIndexB = 0; geometryIndexB < geometryB.Length; geometryIndexB++)
                    {
                        GJKOutput gjkOutput = collisionEngine.Execute(
                            geometryA[geometryIndexA],
                            geometryB[geometryIndexB]);

                        CollisionPointStructure collision = NarrowPhaseCollisionDetection(
                            gjkOutput,
                            geometryA[geometryIndexA],
                            geometryB[geometryIndexB],
                            indexA,
                            indexB);

                        if (collision != null)
                            collisionPointStructure.Add(collision);
                    }
                }
            }
            else if (A is ISoftShape && 
                     B is ISoftShape)
            {
                List<CollisionPointBaseStructure> baseCollisionList = new List<CollisionPointBaseStructure>();

                ISoftShape softShapeA = (ISoftShape)A;
                ISoftShape softShapeB = (ISoftShape)B;

                baseCollisionList.AddRange(softBodyCollisionDetection.SelfSoftBodyCollisionDetection(softShapeA, CollisionDistance));
                baseCollisionList.AddRange(softBodyCollisionDetection.SelfSoftBodyCollisionDetection(softShapeB, CollisionDistance));

                baseCollisionList.AddRange(softBodyCollisionDetection.SoftVsSoftBodyCollisionDetection(softShapeA, softShapeB, CollisionDistance));
                
            }
                        
            return collisionPointStructure;
        }

        #endregion

    }
}
