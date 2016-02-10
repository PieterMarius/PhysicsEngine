using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using PhysicsEngineMathUtility;
using ObjectDefinition;

namespace CollisionEngine
{
	public class CollisionDetectionEngine: ICollisionEngine
	{
		#region Private Fields

		private GJK collisionEngine;
		private EPA compenetrationCollisionEngine;
		private CollisionEngineParameters collisionEngineParameters;

		#endregion

		#region Sweep And Prune fields

		private List<double> vectorX;
		private List<double> vectorY;
		private List<double> vectorZ;

		#endregion

		#region Constructor

		public CollisionDetectionEngine (
			CollisionEngineParameters collisionEngineParameters)
		{
			this.collisionEngineParameters = collisionEngineParameters;

			this.collisionEngine = new GJK (
				collisionEngineParameters.MaxGJKIteration,
				collisionEngineParameters.Precision,
				collisionEngineParameters.GJKManifoldTolerance,
				collisionEngineParameters.ManifoldPointNumber);

			this.compenetrationCollisionEngine = new EPA (
				collisionEngineParameters.MaxEPAIteration,
				collisionEngineParameters.Precision,
				collisionEngineParameters.EPAManifoldTolerance,
				collisionEngineParameters.ManifoldPointNumber);
		}

		#endregion

		#region Interface ICollisionEngine

		/// <summary>
		/// Runs the test collision. 
		/// Every time this method is called the previuos call is reset.
		/// </summary>
		/// <returns><c>true</c>, if test collision was run, <c>false</c> otherwise.</returns>
		/// <param name="A">A.</param>
		/// <param name="B">B.</param>
		public List<CollisionPointStructure> RunTestCollision(
			ObjectGeometry[] objects,
			double minDistance)
		{
			return this.bruteForceBroadPhase (
				objects,
				minDistance);
		}

		/// <summary>
		/// Gets the engine parameters.
		/// </summary>
		/// <returns>The engine parameters.</returns>
		public CollisionEngineParameters GetEngineParameters()
		{
			return this.collisionEngineParameters;
		}

		#endregion

		#region Public Methods

		#endregion

		#region Private Methods

		private CollisionPointStructure narrowPhase(
			ObjectGeometry A,
			ObjectGeometry B,
			int indexA,
			int indexB,
			double minDistance)
		{
			
			GJKOutput gjkOutput = this.collisionEngine.ExecuteGJKAlgorithm (A, B);

			if (!gjkOutput.Intersection &&
				gjkOutput.CollisionDistance <= minDistance)
			{

  				ManifoldPointsGenerator mpg = new ManifoldPointsGenerator (
					                              this.collisionEngineParameters.ManifoldPointNumber,
					                              this.collisionEngineParameters.GJKManifoldTolerance,
					                              0.000001);

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints (
					                                           A,
					                                           B,
					                                           gjkOutput.CollisionPoint,
					                                           gjkOutput.CollisionNormal);

				return new CollisionPointStructure (
					indexA,
					indexB,
					gjkOutput.CollisionDistance,
					gjkOutput.Intersection,
					0.0,
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

				EPAOutput epaOutput = this.compenetrationCollisionEngine.GetCompenetrationDistance (
					                      A,
					                      B,
					                      startTriangle);

				ManifoldPointsGenerator mpg = new ManifoldPointsGenerator (
					                              this.collisionEngineParameters.ManifoldPointNumber,
					                              this.collisionEngineParameters.EPAManifoldTolerance,
					                              0.00001);

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints (
					                                           A,
					                                           B,
					                                           epaOutput.CollisionPoint,
					                                           epaOutput.CollisionPoint.collisionNormal);

				return new CollisionPointStructure (
					indexA,
					indexB,
					-1.0,
					gjkOutput.Intersection,
					epaOutput.CompenetrationDistance,
					epaOutput.CollisionPoint,
					collisionPointsList.ToArray ());
			} 

			return null;
		}

		private List<CollisionPointStructure> bruteForceBroadPhase(
			ObjectGeometry[] objects,
			double minDistance)
		{
			
			List<CollisionPointStructure> result = new List<CollisionPointStructure> ();

			object lockMe = new object();

			Parallel.For (0, 
				objects.Length, 
				new ParallelOptions { MaxDegreeOfParallelism = this.collisionEngineParameters.MaxThreadNumber }, 
				i => {
		
					for (int j = i + 1; j < objects.Length; j++) {
						CollisionPointStructure collisionPointStruct = this.narrowPhase (
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
				});
			
			return result;
		}

		private List<CollisionPointStructure> sweepAndPruneBroadPhase()
		{
			
			return null;
		}

		#endregion

	}
}

