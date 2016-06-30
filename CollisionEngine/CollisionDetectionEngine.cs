﻿using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public class CollisionDetectionEngine: ICollisionEngine
	{
		#region Private Fields

		GJK collisionEngine;
		EPA compenetrationCollisionEngine;
		readonly CollisionEngineParameters collisionEngineParameters;
		readonly SweepAndPruneEngine sweepAndPruneEngine;

		#endregion

		#region Constructor

		public CollisionDetectionEngine (
			CollisionEngineParameters collisionEngineParameters)
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
		}

		#endregion

		#region Public Methods

		#region Interface ICollisionEngine

		/// <summary>
		/// Runs the test collision.
		/// </summary>
		/// <returns>The test collision.</returns>
		/// <param name="objects">Objects.</param>
		/// <param name="minDistance">Minimum distance.</param>
		public List<CollisionPointStructure> RunTestCollision(
			ObjectGeometry[] objects,
			double minDistance)
		{
			if (collisionEngineParameters.ActivateSweepAndPrune) 
			{
				return SweepAndPruneBroadPhase (
					objects,
					minDistance);
			} 
			else 
			{
				return BruteForceBroadPhase (
					objects,
					minDistance);
			}
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

		private CollisionPointStructure NarrowPhase(
			ObjectGeometry A,
			ObjectGeometry B,
			int indexA,
			int indexB,
			double minDistance)
		{
			
			GJKOutput gjkOutput = collisionEngine.ExecuteGJKAlgorithm (A, B);

			if (!gjkOutput.Intersection &&
				gjkOutput.CollisionDistance <= minDistance)
			{

  				var mpg = new ManifoldPointsGenerator (
					                              collisionEngineParameters.ManifoldPointNumber,
					                              collisionEngineParameters.GJKManifoldTolerance,
					                              0.000001);

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints (
					                                           A,
					                                           B,
					                                           gjkOutput.CollisionPoint,
					                                           gjkOutput.CollisionNormal);

				return new CollisionPointStructure (
					indexA,
					indexB,
					gjkOutput.Intersection,
					gjkOutput.CollisionDistance,
					gjkOutput.CollisionPoint,
					collisionPointsList.ToArray ());
			} 

			if (gjkOutput.Intersection)
			{
				Support[] startTriangle = new Support[4];
				startTriangle [0] = gjkOutput.MinSimplex.Support [0];
				startTriangle [1] = gjkOutput.MinSimplex.Support [1];
				startTriangle [2] = gjkOutput.MinSimplex.Support [2];
				startTriangle [3] = gjkOutput.MinSimplex.Support [3];

				EPAOutput epaOutput = compenetrationCollisionEngine.GetCompenetrationDistance (
					                      A,
					                      B,
					                      startTriangle);

				var mpg = new ManifoldPointsGenerator (
					                              collisionEngineParameters.ManifoldPointNumber,
					                              collisionEngineParameters.EPAManifoldTolerance,
					                              0.00001);

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints (
					                                           A,
					                                           B,
					                                           epaOutput.CollisionPoint,
					                                           epaOutput.CollisionPoint.collisionNormal);

				return new CollisionPointStructure (
					indexA,
					indexB,
					gjkOutput.Intersection,
					epaOutput.CompenetrationDistance,
					epaOutput.CollisionPoint,
					collisionPointsList.ToArray ());
			} 

			return null;
		}

		private List<CollisionPointStructure> BruteForceBroadPhase(
			ObjectGeometry[] objects,
			double minDistance)
		{
			
			var result = new List<CollisionPointStructure> ();

			var lockMe = new object();

			Parallel.For (0, 
				objects.Length, 
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber }, 
				i => {
					if (objects [i] != null) {
						for (int j = i + 1; j < objects.Length; j++) {
							if (objects [j] != null) {
								CollisionPointStructure collisionPointStruct = NarrowPhase (
									                                              objects [i], 
									                                              objects [j],
									                                              i,
									                                              j,
									                                              minDistance);

								lock (lockMe) {    
									if (collisionPointStruct != null)
										result.Add (collisionPointStruct);
								}
							}
						}
					}
				});
			
			return result;
		}

		private List<CollisionPointStructure> SweepAndPruneBroadPhase(
			ObjectGeometry[] objects,
			double minDistance)
		{
			var result = new List<CollisionPointStructure> ();

			AABB[] boxs = Array.ConvertAll (objects, item => (item == null) ? null : item.AABBox);

			List<CollisionPair> collisionPair = sweepAndPruneEngine.SweepAndPruneTest (boxs, minDistance);

			var lockMe = new object();

			Parallel.ForEach (
				collisionPair, 
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber }, 
				pair => {
					CollisionPointStructure collisionPointStruct = NarrowPhase (
						objects [pair.objectIndexA], 
						objects [pair.objectIndexB],
						pair.objectIndexA,
						pair.objectIndexB,
						minDistance);

					lock (lockMe) {
						if (collisionPointStruct != null)
							result.Add (collisionPointStruct);
					}
				});

			return result;
		}

		#endregion

	}
}

