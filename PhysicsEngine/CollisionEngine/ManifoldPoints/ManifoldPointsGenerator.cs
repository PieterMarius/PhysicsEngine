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
			Vector3d[] vertexObjA,
			Vector3d[] vertexObjB,
			CollisionPoint collisionPoint)
		{
            if (ManifoldPointNumber > 1)
            {
                List<Vector3d> collisionA = GetNearestPoint(
                    vertexObjA,
                    collisionPoint.CollisionPointA.Vertex,
                    collisionPoint.CollisionNormal);

                List<Vector3d> collisionB = GetNearestPoint(
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
            Vector3d cNormal)
        {
            if(collisionPoints.Count > 2)
            {
                var normal = GeometryUtils.CalculateTriangleNormal(
                    collisionPoints[0].CollisionPointA.Vertex, 
                    collisionPoints[1].CollisionPointA.Vertex,
                    collisionPoints[2].CollisionPointA.Vertex);

                foreach (var point in collisionPoints)
                    point.SetNormal(Vector3d.UniformSign(normal, cNormal));
            }
        }

		/// <summary>
		/// Gets the nearest point from collision point.
		/// </summary>
		/// <returns>The nearest point.</returns>
		/// <param name="shape">Shape.</param>
		/// <param name="collisionPoint">Collision point.</param>z
		private List<Vector3d> GetNearestPoint(
			Vector3d[] vertexObj,
			Vector3d collisionPoint,
			Vector3d planeNormal)
		{
			var collisionPoints = new List<Vector3d> ();

			for (int i = 0; i < vertexObj.Length; i++) 
			{
                Vector3d diff = collisionPoint - vertexObj[i];

                if (diff == Vector3d.Zero())
                {
                    collisionPoints.Add(vertexObj[i]);
                    continue;
                }

				Vector3d nt = Vector3d.Normalize(diff);

                if (Math.Abs(Vector3d.Dot(nt, planeNormal)) < ManifoldPlaneTolerance)
					collisionPoints.Add(vertexObj[i]);
			}

			return collisionPoints;
		}

		private List<CollisionPoint> FindCollisionPoints(
			Vector3d[] ca,
			Vector3d[] cb,
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
				ca = GeometryUtils.TurnVectorClockWise (
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
				cb = GeometryUtils.TurnVectorClockWise (
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
				ca = GeometryUtils.TurnVectorClockWise (
					ca,
                    cp.CollisionNormal);

				cb = GeometryUtils.TurnVectorClockWise (
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
            else
            {
                result.Add(cp);
            }

            if (ManifoldPointNumber == 4 && result.Count > 4)
                result = ExtractFourCollisionPoints(result, cp.CollisionNormal);
            else
                result = ExtractCollisionPoints(result, cp.CollisionNormal);
                        
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
			Vector3d[] ca,
			Vector3d[] cb,
			CollisionPoint initPoint,
			Vector3d na)
		{
			var result = new List<CollisionPoint>();

			if (cb.Length > 2)
			{
				for (int i = 0; i < ca.Length; i++)
				{
					Vector3d project = ca[i] -
						(na * (Vector3d.Dot(na, ca[i]) +
							Vector3d.Dot(na * -1.0, initPoint.CollisionPointB.Vertex)));

					double angle = GeometryUtils.TestPointInsidePolygon(
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
					Vector3d project = cb[i] -
						(na * (Vector3d.Dot(na, cb[i]) +
							Vector3d.Dot(na * -1.0, initPoint.CollisionPointA.Vertex)));

					double angle = GeometryUtils.TestPointInsidePolygon(
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

		private List<CollisionPoint> ExtractCollisionPoints(
            List<CollisionPoint> cpList,
            Vector3d normal)
		{
            if (cpList.Count > ManifoldPointNumber) 
			{
				var center = new Vector3d();
				for (int i = 0; i < cpList.Count; i++)
					center = center + cpList [i].CollisionPointA.Vertex;

				center = center / Convert.ToDouble(cpList.Count);

                int nDirection = ManifoldPointNumber;
                var coneDirection = GetConeDirection(normal, nDirection);
                var arrayList = cpList.ToArray();

                List<CollisionPoint> result = new List<CollisionPoint>();

                for (int i = 0; i < nDirection; i++)
                {
                    int index = GetFarthestPoint(cpList.ToArray(), coneDirection[i]);
                    result.Add(arrayList[index]);
                    cpList.RemoveAt(index);
                }
                
                return result;
			}

			return cpList;
		}

        private Vector3d[] GetConeDirection(
            Vector3d normal,
            int nDirection)
        {
            var coneDirection = new Vector3d[nDirection];

            var tx = new Vector3d();
            var ty = new Vector3d();

            GeometryUtils.ComputeBasis(
                normal,
                ref tx,
                ref ty);

            coneDirection[0] = tx;
            coneDirection[1] = ty;
                        
            double step = Math.PI / nDirection;
            for (int i = 0; i < nDirection; i++)
                coneDirection[i] = Matrix3x3.GetRotationMatrix(normal, step * i) * tx;
            
            return coneDirection;
        }

        private List<CollisionPoint> ExtractFourCollisionPoints(
            List<CollisionPoint> cpList,
            Vector3d normal)
        {
            if (cpList.Count > 4)
            {
                var result = new List<CollisionPoint>();

                //Point 1
                Vector3d A = cpList[0].CollisionPointA.Vertex;
                result.Add(cpList[0]);
                cpList.RemoveAt(0);

                //Point 2
                double maxDist = double.MinValue;
                CollisionPoint farthestPoint = null;
                int index = -1;
                for (int i = 0; i < cpList.Count; i++)
                {
                    double dist = (cpList[i].CollisionPointA.Vertex - A).Length();

                    if (dist > maxDist)
                    {
                        maxDist = dist;
                        farthestPoint = cpList[i];
                        index = i;
                    }
                }
                Vector3d B = farthestPoint.CollisionPointA.Vertex;
                result.Add(farthestPoint);
                cpList.RemoveAt(index);

                //Point 3
                double maxArea = double.MinValue;
                CollisionPoint third = null;
                index = -1;

                for (int i = 0; i < cpList.Count; i++)
                {
                    double area = CalculateArea(A, B, cpList[i].CollisionPointA.Vertex, normal);

                    if(area > maxArea)
                    {
                        maxArea = area;
                        third = cpList[i];
                        index = i;
                    }
                }
                Vector3d C = third.CollisionPointA.Vertex;
                result.Add(third);
                cpList.RemoveAt(index);

                //Point 4
                CollisionPoint fourth = null;
                maxArea = double.MinValue;
                
                for (int i = 0; i < cpList.Count; i++)
                {
                    // A-B-D
                    double area = CalculateArea(A, B, cpList[i].CollisionPointA.Vertex, normal);

                    if (area > maxArea)
                    {
                        maxArea = area;
                        fourth = cpList[i];
                    }

                    // A-C-D
                    area = CalculateArea(A, C, cpList[i].CollisionPointA.Vertex, normal);

                    if (area > maxArea)
                    {
                        maxArea = area;
                        fourth = cpList[i];
                    }

                    // B-C-D
                    area = CalculateArea(B, C, cpList[i].CollisionPointA.Vertex, normal);

                    if (area > maxArea)
                    {
                        maxArea = area;
                        fourth = cpList[i];
                    }
                }

                result.Add(fourth);
                
                return result;

            }
            return cpList;
        }

        private double CalculateArea(
            Vector3d A,
            Vector3d B,
            Vector3d C,
            Vector3d normal)
        {
            Vector3d CA = C - A;
            Vector3d CB = C - B;
            return Math.Abs(0.5 * Vector3d.Dot(CA.Cross(CB), normal));
        }

        private CollisionPoint TestEdgesIntersection(
			Vector3d p1,
			Vector3d p2,
			Vector3d p3,
			Vector3d p4,
			CollisionPoint point)
		{
			var a = new Vector3d ();
			var b = new Vector3d ();
			double mua = 0.0;
			double mub = 0.0;

			if (GeometryUtils.TestEdgesIntersect (
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

        public int GetFarthestPoint(
            CollisionPoint[] vertexObj,
            Vector3d direction)
        {
            int index = 0;
            double maxDot = Vector3d.Dot(vertexObj[index].CollisionPointA.Vertex, direction);

            for (int i = 1; i < vertexObj.Length; i++)
            {
                Vector3d vertex = vertexObj[i].CollisionPointA.Vertex;
                double dot = Vector3d.Dot(vertex, direction);

                if (dot > maxDot)
                {
                    maxDot = dot;
                    index = i;
                }
            }
            return index;
        }


        #endregion
    }
}

