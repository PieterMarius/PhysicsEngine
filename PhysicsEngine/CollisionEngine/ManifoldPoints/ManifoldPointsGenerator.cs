﻿/******************************************************************************
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

using System;
using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal sealed class ManifoldPointsGenerator
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
			ManifoldPointNumber = manifoldPointNumber;
			ManifoldPlaneTolerance = manifoldPlaneTolerance;
			ManifoldStabilizeValue = manifoldStabilizeValue;
		}

		#endregion

		#region Public Methods

		public List<CollisionPoint> GetManifoldPoints(
			Vector3[] vertexObjA,
			Vector3[] vertexObjB,
			CollisionPoint collisionPoint)
		{
            if (ManifoldPointNumber > 1)
            {
                List<Vector3> collisionA = GetNearestPoint(
                    vertexObjA,
                    collisionPoint.CollisionPointA.Vertex,
                    collisionPoint.CollisionNormal);

                List<Vector3> collisionB = GetNearestPoint(
                    vertexObjB,
                    collisionPoint.CollisionPointB.Vertex,
                    collisionPoint.CollisionNormal);

                List<CollisionPoint> collisionPointsList = FindCollisionPoints(
                    collisionA.ToArray(),
                    collisionB.ToArray(),
                    collisionPoint);

                collisionA.Clear();
                collisionB.Clear();

                SetCollisionNormal(collisionPointsList, collisionPoint.CollisionNormal);

                return collisionPointsList;
            }

            return null;
		}

		#endregion

		#region Private Methods

        private void SetCollisionNormal(
            List<CollisionPoint> collisionPoints,
            Vector3 cNormal)
        {
            if(collisionPoints.Count > 2)
            {
                var normal = GeometryUtilities.CalculateNormal(
                    collisionPoints[0].CollisionPointA.Vertex, 
                    collisionPoints[1].CollisionPointA.Vertex,
                    collisionPoints[2].CollisionPointA.Vertex);

                foreach (var point in collisionPoints)
                    point.SetNormal(Vector3.UniformSign(normal, cNormal));
            }
        }

		/// <summary>
		/// Gets the nearest point from collision point.
		/// </summary>
		/// <returns>The nearest point.</returns>
		/// <param name="shape">Shape.</param>
		/// <param name="collisionPoint">Collision point.</param>z
		private List<Vector3> GetNearestPoint(
			Vector3[] vertexObj,
			Vector3 collisionPoint,
			Vector3 planeNormal)
		{
			var collisionPoints = new List<Vector3> ();

			for (int i = 0; i < vertexObj.Length; i++) 
			{
                Vector3 diff = collisionPoint - vertexObj[i];
                if (diff == Vector3.Zero())
                    continue;

				Vector3 nt = Vector3.Normalize(diff);

                if (Math.Abs(Vector3.Dot(nt, planeNormal)) < ManifoldPlaneTolerance)
					collisionPoints.Add(vertexObj[i]);
			}

			return collisionPoints;
		}

		private List<CollisionPoint> FindCollisionPoints(
			Vector3[] ca,
			Vector3[] cb,
			CollisionPoint cp)
		{
			var result = new List<CollisionPoint> ();

			if (ca.Length == 2 && cb.Length == 2) 
			{
				CollisionPoint collisionP = TestEdgesIntersection (
					ca [0],
					ca [1],
					cb [0],
					cb [1],
					cp);

				if (collisionP != null)
					result.Add (collisionP);

			} 
			else if (ca.Length > 2 && cb.Length == 2) 
			{
				ca = GeometryUtilities.TurnVectorClockWise (
					ca,
                    cp.CollisionNormal);

				result.AddRange(TestPointIsOnPlane (
					ca,
					cb,
					cp,
                    cp.CollisionNormal));

				if (result.Count < ca.Length) {
					for (int i = 0; i < ca.Length; i++) {

						CollisionPoint collisionP = TestEdgesIntersection (
							ca [i],
							ca [(i + 1) % ca.Length],
							cb [0],
							cb [1],
							cp);

						if (collisionP != null)
							result.Add (collisionP);
					}
				}
			} 
			else if (ca.Length == 2 && cb.Length > 2) 
			{
				cb = GeometryUtilities.TurnVectorClockWise (
					cb,
                    cp.CollisionNormal);

				result.AddRange(TestPointIsOnPlane (
					ca,
					cb,
					cp,
                    cp.CollisionNormal));

				if (result.Count < cb.Length) {
					for (int i = 0; i < cb.Length; i++) {

						CollisionPoint collisionP = TestEdgesIntersection (
							ca [0],
							ca [1],
							cb [i],
							cb [(i + 1) % cb.Length],
							cp);

						if (collisionP != null)
							result.Add (collisionP);
					}
				}
			} 
			else if (ca.Length > 2 && cb.Length > 2) 
			{
				ca = GeometryUtilities.TurnVectorClockWise (
					ca,
                    cp.CollisionNormal);

				cb = GeometryUtilities.TurnVectorClockWise (
					cb,
                    cp.CollisionNormal);

				result.AddRange(TestPointIsOnPlane (
					ca,
					cb,
					cp,
                    cp.CollisionNormal));

				for (int i = 0; i < ca.Length; i++) {
					for (int j = 0; j < cb.Length; j++) {

						CollisionPoint collisionP = TestEdgesIntersection (
							ca [i],
							ca [(i + 1) % ca.Length],
							cb [j],
							cb [(j + 1) % cb.Length],
							cp);
						
						if (collisionP != null)
							result.Add (collisionP);
					}
				}
			}

            result = PruneCollisionPoints (result);

			result.Add(cp);

			return result;
		}

		/// <summary>
		/// Tests the point is on plane.
		/// </summary>
		/// <returns>The point is on plane.</returns>
		/// <param name="ca">Ca.</param>
		/// <param name="cb">Cb.</param>
		/// <param name="initPoint">Init point.</param>
		/// <param name="na">Na.</param>
		private List<CollisionPoint> TestPointIsOnPlane(
			Vector3[] ca,
			Vector3[] cb,
			CollisionPoint initPoint,
			Vector3 na)
		{
			var result = new List<CollisionPoint>();

			if (cb.Length > 2)
			{
				for (int i = 0; i < ca.Length; i++)
				{
					Vector3 project = ca[i] -
						(na * (Vector3.Dot(na, ca[i]) +
							Vector3.Dot(na * -1.0, initPoint.CollisionPointB.Vertex)));

					double angle = GeometryUtilities.TestPointInsidePolygon(
						cb,
						project,
						na,
						initPoint.CollisionPointB.Vertex);

					//Inserito il minore per gestire problemi di approssimazione
					if (angle + ManifoldStabilizeValue >= ConstValues.PI2)
					{
						var cp = new CollisionPoint(
							new VertexProperties(ca[i]),
							new VertexProperties(project),
							na,
                            0.0,
                            false);
						result.Add(cp);
					}
				}
			}

			if (ca.Length > 2)
			{
				for (int i = 0; i < cb.Length; i++)
				{
					Vector3 project = cb[i] -
						(na * (Vector3.Dot(na, cb[i]) +
							Vector3.Dot(na * -1.0, initPoint.CollisionPointA.Vertex)));

					double angle = GeometryUtilities.TestPointInsidePolygon(
						ca,
						project,
						na,
						initPoint.CollisionPointA.Vertex);

					if (angle + ManifoldStabilizeValue >= ConstValues.PI2)
					{
						var cp = new CollisionPoint(
							new VertexProperties(project),
							new VertexProperties(cb[i]),
							na, 
                            0.0,
                            false);
						result.Add(cp);
					}
				}
			}

			return result;
		}

		//Pulisco il vettore da punti ridondanti
		//al fine della simulazione non necessito più di 4 punti di collisione
		private List<CollisionPoint> PruneCollisionPoints(
			List<CollisionPoint> cpList)
		{
			if (cpList.Count > ManifoldPointNumber - 1) 
			{
				var center = new Vector3();
				for (int i = 0; i < cpList.Count; i++)
					center = center + cpList [i].CollisionPointA.Vertex;

				center = center / Convert.ToDouble(cpList.Count);

				while (cpList.Count > ManifoldPointNumber - 1) 
				{
					int index = 0;
					double min = Vector3.Length (cpList [0].CollisionPointA.Vertex - center);
					for (int i = 1; i < cpList.Count; i++) 
					{
						double minx = Vector3.Length (cpList [i].CollisionPointA.Vertex - center);
						if (minx < min) 
						{
							min = minx;
							index = i;
						}
					}
					cpList.RemoveAt (index);
				}
			}

			return cpList;
		}

		private CollisionPoint TestEdgesIntersection(
			Vector3 p1,
			Vector3 p2,
			Vector3 p3,
			Vector3 p4,
			CollisionPoint point)
		{
			var a = new Vector3 ();
			var b = new Vector3 ();
			double mua = 0.0;
			double mub = 0.0;

			if (GeometryUtilities.TestEdgesIntersect (
				p1,
				p2,
				p3,
				p4,
				ref a,
				ref b,
				ref mua,
				ref mub)) 
			{
				if (!(mua < 0.0 || mua > 1.0 || mub < 0.0 || mub > 1.0))
					return new CollisionPoint (
						new VertexProperties(a),
						new VertexProperties(b),
						point.CollisionNormal,
                        0.0,
                        false);
			}

			return null;
		}


		#endregion
	}
}

