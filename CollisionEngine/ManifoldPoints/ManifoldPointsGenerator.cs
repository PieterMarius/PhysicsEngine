﻿using System;
using System.Collections.Generic;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public class ManifoldPointsGenerator
	{
		#region Properties

		public int ManifoldPointNumber { get; private set; }
		public double ManifoldPlaneTolerance { get; private set; }
		public double ManifoldStabilizeValue { get; private set; }

		#endregion

		#region Constructor

		public ManifoldPointsGenerator (
			int manifoldPointNumber,
			double manifoldPlaneTolerance,
			double manifoldStabilizeValue)
		{
			this.ManifoldPointNumber = manifoldPointNumber;
			this.ManifoldPlaneTolerance = manifoldPlaneTolerance;
			this.ManifoldStabilizeValue = manifoldStabilizeValue;
		}

		#endregion

		#region Public Methods

		public List<CollisionPoint> GetManifoldPoints(
			ObjectGeometry objectA, 
			ObjectGeometry objectB,
			CollisionPoint collisionPoint,
			Vector3 collisionNormal)
		{
			List<Vector3> collisionA = this.getNearestPoint (
				objectA,
				collisionPoint.collisionPointA,
				collisionNormal,
				this.ManifoldPlaneTolerance);

			List<Vector3> collisionB = this.getNearestPoint (
				objectB,
				collisionPoint.collisionPointB,
				collisionNormal,
				this.ManifoldPlaneTolerance);

			List<CollisionPoint> collisionPointsList = this.findCollisionPoints (
				collisionA.ToArray (),
				collisionB.ToArray (),
				collisionNormal,
				collisionPoint);

			collisionA.Clear ();
			collisionB.Clear ();

			return collisionPointsList;

		}

		#endregion

		#region Private Methods

		/// <summary>
		/// Gets the nearest point from collision point.
		/// </summary>
		/// <returns>The nearest point.</returns>
		/// <param name="shape">Shape.</param>
		/// <param name="collisionPoint">Collision point.</param>
		/// <param name="direction">Direction.</param>
		/// <param name="tolerance">Tolerance.</param>
		private List<Vector3> getNearestPoint(
			ObjectGeometry shape,
			Vector3 collisionPoint,
			Vector3 planeNormal,
			double tolerance)
		{
			List<Vector3> collisionPoints = new List<Vector3> ();

			Vector3 normal = Vector3.Normalize(planeNormal);
			for (int i = 0; i < shape.VertexPosition.Length; i++) 
			{
				Vector3 nt = Vector3.Normalize (shape.VertexPosition [i] - collisionPoint);
				if (Math.Abs (Vector3.Dot (nt, normal)) < tolerance)
					collisionPoints.Add (shape.VertexPosition [i]);
			}

			return collisionPoints;
		}

		/// <summary>
		/// Tests if point is on plane. 
		/// Proietto i punti di ca sul piano formato dai punti di cb e verifico
		/// che siano all'interno di esso.
		/// </summary>
		/// <param name="ca">Ca.</param>
		/// <param name="na">Na.</param>
		/// <param name="cb">Cb.</param>
		/// <param name="initp">Initp.</param>
		private void testIfPointIsOnPlane(
			Vector3[] ca,
			Vector3[] cb,
			CollisionPoint initPoint,
			Vector3 na,
			ref List<CollisionPoint> result)
		{
			if (cb.Length > 2) 
			{
				for (int i = 0; i < ca.Length; i++) 
				{
					Vector3 project = ca [i] -
						(na * (Vector3.Dot (na, ca [i]) +
							Vector3.Dot (na * -1.0, initPoint.collisionPointB)));

					double angle = GeometryUtilities.TestPointInsidePolygon (
						cb,
						project,
						na,
						initPoint.collisionPointB);

					//Inserito il minore per gestire problemi di approssimazione
					if (angle + this.ManifoldStabilizeValue >= 2.0 * Math.PI) 
					{
						CollisionPoint cp = new CollisionPoint (
							ca [i],
							project,
							na);
						result.Add (cp);
					}
				}
			}

			if (ca.Length > 2) 
			{
				for (int i = 0; i < cb.Length; i++) 
				{
					Vector3 project = cb [i] -
						(na * (Vector3.Dot (na, cb[i]) +
							Vector3.Dot (na * -1.0, initPoint.collisionPointA)));

					double angle = GeometryUtilities.TestPointInsidePolygon (
						ca,
						project,
						na,
						initPoint.collisionPointA);

					if (angle + this.ManifoldStabilizeValue >= 2.0 * Math.PI) 
					{

						CollisionPoint cp = new CollisionPoint (
							project,
							cb [i],
							na);
						result.Add (cp);
					}
				}
			}
		}

		private List<CollisionPoint> findCollisionPoints(
			Vector3[] ca,
			Vector3[] cb,
			Vector3 vectorDistance,
			CollisionPoint cp)
		{
			List<CollisionPoint> result = new List<CollisionPoint> ();

			if (ca.Length == 2 && cb.Length == 2) 
			{
				CollisionPoint collisionP = this.testLineIntersection (
					ca [0],
					ca [1],
					cb [0],
					cb [1],
					cp.collisionNormal);

				if (collisionP != null)
					result.Add (collisionP);

			} 
			else if (ca.Length > 2 && cb.Length == 2) 
			{
				ca = GeometryUtilities.TurnVectorClockWise (
					ca,
					vectorDistance);

				this.testIfPointIsOnPlane (
					ca,
					cb,
					cp,
					vectorDistance,
					ref result);

				if (result.Count < ca.Length) {
					for (int i = 0; i < ca.Length; i++) {

						CollisionPoint collisionP = this.testLineIntersection (
							ca [i],
							ca [(i + 1) % ca.Length],
							cb [0],
							cb [1],
							cp.collisionNormal);

						if (collisionP != null)
							result.Add (collisionP);
					}
				}

			} 
			else if (ca.Length == 2 && cb.Length > 2) 
			{
				cb = GeometryUtilities.TurnVectorClockWise (
					cb,
					vectorDistance);

				this.testIfPointIsOnPlane (
					ca,
					cb,
					cp,
					vectorDistance,
					ref result);

				if (result.Count < cb.Length) {
					for (int i = 0; i < cb.Length; i++) {

						CollisionPoint collisionP = this.testLineIntersection (
							ca [0],
							ca [1],
							cb [i],
							cb [(i + 1) % cb.Length],
							cp.collisionNormal);

						if (collisionP != null)
							result.Add (collisionP);
					}
				}
			} 
			else if (ca.Length > 2 && cb.Length > 2) 
			{
				ca = GeometryUtilities.TurnVectorClockWise (
					ca,
					vectorDistance);

				cb = GeometryUtilities.TurnVectorClockWise (
					cb,
					vectorDistance);

				this.testIfPointIsOnPlane (
					ca,
					cb,
					cp,
					vectorDistance,
					ref result);

				for (int i = 0; i < ca.Length; i++) {
					for (int j = 0; j < cb.Length; j++) {

						CollisionPoint collisionP = this.testLineIntersection (
							ca [i],
							ca [(i + 1) % ca.Length],
							cb [j],
							cb [(j + 1) % cb.Length],
							cp.collisionNormal);
						
						if (collisionP != null)
							result.Add (collisionP);
					}
				}

			}

			if (result.Count < 1)
				result.Add (cp);

			result = this.pruneCollisionPoints (result);

			return result;
		}

		//Pulisco il vettore da punti ridondanti
		//al fine della simulazione non necessito più di 4 punti di collisione
		private List<CollisionPoint> pruneCollisionPoints(
			List<CollisionPoint> cpList)
		{
			if (cpList.Count > this.ManifoldPointNumber) 
			{
				Vector3 center = new Vector3();
				for (int i = 0; i < cpList.Count; i++)
					center = center + cpList [i].collisionPointA;

				center = center / Convert.ToDouble(cpList.Count);

				while (cpList.Count > this.ManifoldPointNumber) 
				{
					int index = 0;
					double min = Vector3.Length (cpList [0].collisionPointA - center);
					for (int i = 1; i < cpList.Count; i++) 
					{
						double minx = Vector3.Length (cpList [i].collisionPointA - center);
						if (minx < min) {
							min = minx;
							index = i;
						}
					}
					cpList.RemoveAt (index);
				}
			}

			return cpList;
		}

		private CollisionPoint testLineIntersection(
			Vector3 p1,
			Vector3 p2,
			Vector3 p3,
			Vector3 p4,
			Vector3 normal)
		{
			Vector3 a = new Vector3 ();
			Vector3 b = new Vector3 ();
			double mua = 0.0;
			double mub = 0.0;

			if (GeometryUtilities.TestLinesIntersect (
				p1,
				p2,
				p3,
				p4,
				ref a,
				ref b,
				ref mua,
				ref mub)) 
			{
				if(!(mua < 0.0 || mua > 1.0 || mub < 0.0 || mub > 1.0))
					return new CollisionPoint( a, b, normal);
			}

			return null;
		}


		#endregion
	}
}

