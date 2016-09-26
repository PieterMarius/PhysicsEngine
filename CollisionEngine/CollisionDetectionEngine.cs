using System;
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
		public List<CollisionPointStructure> Execute(
			ObjectGeometry[] objects,
			double minDistance)
		{
			if (collisionEngineParameters.ActivateSweepAndPrune) 
			{
				return SweepAndPruneBroadPhase(
					objects,
					minDistance);
			} 
			else 
			{
				return BruteForceBroadPhase(
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

		public CollisionPointStructure GetIntersectionDistance(
			ObjectGeometry objectA,
			ObjectGeometry objectB)
		{
			GJKOutput gjkOutput = collisionEngine.Execute(objectA, objectB);

			if (!gjkOutput.Intersection)
			{
				return new CollisionPointStructure(
					-1,
					-1,
					gjkOutput.Intersection,
					gjkOutput.CollisionDistance,
					gjkOutput.CollisionPoint,
					null);
			}
			else
			{
				Support[] startTriangle = new Support[4];
				Array.Copy(gjkOutput.MinSimplex.Support, startTriangle, startTriangle.Length);

				EPAOutput epaOutput = compenetrationCollisionEngine.Execute(
										  objectA,
										  objectB,
										  startTriangle);

				return new CollisionPointStructure(
					-1,
					-1,
					gjkOutput.Intersection,
					epaOutput.CompenetrationDistance,
					epaOutput.CollisionPoint,
					null);
			}
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
			GJKOutput gjkOutput = collisionEngine.Execute (A, B);

			Console.WriteLine("Distance " + gjkOutput.CollisionDistance);

			if (!gjkOutput.Intersection &&
				gjkOutput.CollisionDistance <= minDistance)
			{
  				var mpg = new ManifoldPointsGenerator (
					                              collisionEngineParameters.ManifoldPointNumber,
					                              collisionEngineParameters.GJKManifoldTolerance,
					                              collisionEngineParameters.ManifoldProjectionTolerance);

				Console.WriteLine("check point");
				Console.WriteLine("normpal " + gjkOutput.CollisionNormal.x + " " + gjkOutput.CollisionNormal.y + " " + gjkOutput.CollisionNormal.z);
				if (Math.Abs(gjkOutput.CollisionNormal.y) < 0.9999)
					Console.WriteLine("wrong normal " + gjkOutput.CollisionNormal.x + " " + gjkOutput.CollisionNormal.y + " " + gjkOutput.CollisionNormal.z);
				
				if (gjkOutput.CollisionNormal.Length() < 1E-15)
					return null;

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints (
					                                           A,
					                                           B,
					                                           gjkOutput.CollisionPoint);

				Console.WriteLine("n collision poin " + collisionPointsList.Count);

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
				Array.Copy(gjkOutput.MinSimplex.Support, startTriangle, startTriangle.Length);

				EPAOutput epaOutput = compenetrationCollisionEngine.Execute (
					                      A,
					                      B,
					                      startTriangle);

				Console.WriteLine("compenetration " + epaOutput.CompenetrationDistance);

				//Console.WriteLine("normal " + epaOutput.CollisionPoint.CollisionNormal.x + " " + epaOutput.CollisionPoint.CollisionNormal.y + " " + epaOutput.CollisionPoint.CollisionNormal.z);
				if (Math.Abs(epaOutput.CollisionPoint.CollisionNormal.y) < 0.9999)
					Console.WriteLine("wrong epa normal " + epaOutput.CollisionPoint.CollisionNormal.x + " " + epaOutput.CollisionPoint.CollisionNormal.y + " " + epaOutput.CollisionPoint.CollisionNormal.z);


				if (epaOutput.CollisionPoint.CollisionNormal.Length() < 1E-15)
					return null;

				var mpg = new ManifoldPointsGenerator(
												  collisionEngineParameters.ManifoldPointNumber,
												  collisionEngineParameters.EPAManifoldTolerance,
												  collisionEngineParameters.ManifoldProjectionTolerance);

				List<CollisionPoint> collisionPointsList = mpg.GetManifoldPoints(
															   A,
															   B,
															   epaOutput.CollisionPoint);

				Console.WriteLine("n collision point 1 " + collisionPointsList.Count);

				return new CollisionPointStructure(
					indexA,
					indexB,
					gjkOutput.Intersection,
					epaOutput.CompenetrationDistance,
					epaOutput.CollisionPoint,
					collisionPointsList.ToArray());		
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

			List<CollisionPair> collisionPair = sweepAndPruneEngine.Execute (boxs, minDistance);

			var lockMe = new object();

			Parallel.ForEach (
				collisionPair, 
				new ParallelOptions { MaxDegreeOfParallelism = collisionEngineParameters.MaxThreadNumber }, 
				pair => {
					CollisionPointStructure collisionPointStruct = NarrowPhase(
						objects[pair.objectIndexA],
						objects[pair.objectIndexB],
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

