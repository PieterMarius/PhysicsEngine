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

using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal sealed class GJK
	{

		#region Settings Variables

		public int MaxIterations { get; private set; }
		public double Precision { get; private set; }
		public double GJKManifoldTolerance { get; private set;}
		public int ManifoldPointNumber { get; private set;}

		private readonly Vector3d origin = new Vector3d();
		private readonly double constTolerance = 0.0000001;
		
		#endregion

		#region Constructors

		public GJK (
			int maxIterations,
			double precision,
			double gjkTolerance,
			int manifoldPointNumber)
		{
			MaxIterations = maxIterations;
			Precision = precision;
			GJKManifoldTolerance = gjkTolerance;
			ManifoldPointNumber = manifoldPointNumber;
		}

		#endregion

		#region Private Methods

		/// <summary>
		/// Gets the farthest vertex point between two input objects.
		/// </summary>
		/// <returns>The farthest point.</returns>
		/// <param name="objA">Object a.</param>
		/// <param name="objB">Object b.</param>
		private Support GetFarthestPoint(
			VertexProperties[] vertexObjA,
			VertexProperties[] vertexObjB,
			int startIndex)
		{
			int indexA = 0;
			int indexB = startIndex;
			
			return new Support(
				vertexObjA[indexA].Vertex - vertexObjB[indexB].Vertex,
				indexA,
				indexB);
		}

		private Vector3d GetDirectionOnSimplex2(Simplex simplex)
		{
			Vector3d simplexAB = simplex.Support [1].s - simplex.Support [0].s;
			Vector3d simplexAO = simplex.Support [0].s * - 1.0;

			return Vector3d.Cross(
					Vector3d.Cross(simplexAB, simplexAO), 
					simplexAB);
		}

		private Vector3d GetMinDistance(
			ref List<SupportTriangle> triangles,
			Vector3d point,
			ref int minTriangleIndex)
		{
			var result = new Vector3d();
			var distanceBuf = new Vector3d();
			double s = 0; double t = 0;

			var buffer = new SupportTriangle();

			double minDistance = double.MaxValue;

			for (int i = 0; i < triangles.Count; i++)
			{
				buffer = triangles[i];

				if (!GeometryUtils.TestCollinearity(
						buffer.A.s,
						buffer.B.s,
						buffer.C.s))
				{
					distanceBuf = GeometryUtils.GetPointTriangleIntersection(
						buffer.A.s,
						buffer.B.s,
						buffer.C.s,
						point,
						ref s,
						ref t).Value;

					buffer.SetValueS(s);
					buffer.SetValueT(t);
				}
				else
				    continue;
				

				triangles[i] = buffer;

				double distance = Vector3d.Length(distanceBuf);

				if (distance < minDistance)
				{
					minDistance = distance;
					minTriangleIndex = i;
					result = distanceBuf;
				}
			}

			return result;
		}

		/// <summary>
		/// Ritorna la distanza se distanza 0.0f allora vi è intersezione o compenetrazione tra gli oggetti, 
		/// se distanza 0.0 allora i due oggetti non collidono
		/// </summary>
		/// <returns>The GJK algorithm.</returns>
		/// <param name="shape1">Shape1.</param>
		/// <param name="shape2">Shape2.</param>
		/// <param name="cp">Cp.</param>
		/// <param name="isIntersection">If set to <c>true</c> is itersection.</param>
		private double ExecuteGJKAlgorithm(
			VertexProperties[] vertexShape1,
			VertexProperties[] vertexShape2,
			ref Vector3d collisionNormal,
			ref CollisionPoint cp,
			ref List<SupportTriangle> triangles,
			ref Vector3d centroid,
			ref bool isIntersection)
		{
			double minDistance = double.MaxValue;
			int minTriangleIndex = -1;
			var result = new EngineCollisionPoint();
			var oldDirection = new Vector3d();
			var simplex = new Simplex();

			//Primo punto del simplex
			simplex.Support.Add(GetFarthestPoint(vertexShape1, vertexShape2, vertexShape2.Length / 2));

			//Secondo punto del simplex
			Vector3d direction = Vector3d.Normalize(simplex.Support[0].s * -1.0);
			if (!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(vertexShape1, vertexShape2, direction)))
				return -1.0;

			//Terzo punto del simplex
			direction = Vector3d.Normalize(GetDirectionOnSimplex2(simplex));
			if(!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(vertexShape1, vertexShape2, direction)))
				return -1.0;

			//Quarto punto del simplex
			direction = Vector3d.Normalize(GeometryUtils.CalculateNormal(
				simplex.Support[0].s,
				simplex.Support[1].s,
				simplex.Support[2].s));

			if (!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(vertexShape1, vertexShape2, direction)))
				simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(vertexShape1, vertexShape2, -1.0 * direction));

			//Costruisco il poliedro
			centroid = Helper.SetStartTriangle(
									ref triangles,
									simplex.Support.ToArray());

			//Verifico che l'origine sia contenuta nel poliedro
			if (Helper.IsInConvexPoly(origin, triangles))
			{
				isIntersection = true;
				return -1.0;
			}

			Vector3d triangleDistance = GetMinDistance(ref triangles, origin, ref minTriangleIndex);

			result.SetDist(triangleDistance);
			result.SetNormal(Vector3d.Normalize(triangleDistance));
			Helper.GetVertexFromMinkowsky(triangles[minTriangleIndex], vertexShape1, vertexShape2, ref result);

			minDistance = triangleDistance.Length();

			for (int i = 0; i < MaxIterations; i++) 
			{
				direction = -1.0 * triangleDistance.Normalize();

				if (Vector3d.Length(direction) < constTolerance)
				{
					direction = origin - centroid;
				}

				if (direction == oldDirection)
					break;

				oldDirection = direction;

				if (!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(vertexShape1, vertexShape2, direction)))
				{
					for (int j = 0; j < triangles.Count; j++)
					{
						direction = triangles[j].Normal;
						if (!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(vertexShape1, vertexShape2, direction)))
						{
							if (simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(vertexShape1, vertexShape2, -1.0 * direction)))
							   break;
							
							continue;
						}
						break;
					}
				}

				triangles = Helper.AddPointToConvexPolygon(triangles, simplex.Support[simplex.Support.Count - 1], centroid);

                //Verifico che l'origine sia contenuta nel poliedro
                if (Helper.IsInConvexPoly(origin, triangles))
                {
                    isIntersection = true;
                    return -1.0;
                }

				triangleDistance = GetMinDistance(ref triangles, origin, ref minTriangleIndex);

				double mod = triangleDistance.Length();

				if (mod < minDistance)
				{
					result.SetDist(triangleDistance);
                    result.SetNormal(triangles[minTriangleIndex].Normal);

                    Helper.GetVertexFromMinkowsky(triangles[minTriangleIndex], vertexShape1, vertexShape2, ref result);

					minDistance = mod;
				}
			}

			collisionNormal = -1.0 * result.Normal;

			cp = new CollisionPoint(
				result.A,
				result.B,
				collisionNormal,
                0.0,
                false);
			
			return minDistance;
		}
					
		#endregion

		#region Public Methods

		/// <summary>
		/// Detects the collision.
		/// </summary>
		/// <returns>The collision.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		public GJKOutput Execute(
			VertexProperties[] vertexObjA,
			VertexProperties[] vertexObjB)
		{
			var collisionPoint = new CollisionPoint();
			var collisionNormal = new Vector3d();
			var supportTriangles = new List<SupportTriangle>();
			var centroid = new Vector3d();
			bool isIntersection = false;
			
			//TODO refactoring
			double collisionDistance = ExecuteGJKAlgorithm (
										  vertexObjA,
										  vertexObjB,
										  ref collisionNormal,
										  ref collisionPoint,
										  ref supportTriangles,
										  ref centroid,
										  ref isIntersection);

            return new GJKOutput (
				collisionDistance,
				collisionPoint,
				collisionNormal,
				centroid,
				isIntersection,
				supportTriangles);
		}
			

		#endregion

	}
}

