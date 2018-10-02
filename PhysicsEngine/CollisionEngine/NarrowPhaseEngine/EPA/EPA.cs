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
	internal sealed class EPA
	{
		#region Settings Variable

		public int MaxIterations { get; private set; }
		public double Precision { get; private set; }
		public double EPAManifoldTolerance { get; private set;}
		public int ManifoldPointNumber { get; private set;}

		readonly Vector3d origin = new Vector3d();

		readonly double constTolerance = 0.0000001;

		#endregion

		#region Constructor

		public EPA (
			int maxIterations,
			double precision,
			double epaTolerance,
			int manifoldPointNumber)
		{
			MaxIterations = maxIterations;
			Precision = precision;
			EPAManifoldTolerance = epaTolerance;
			ManifoldPointNumber = manifoldPointNumber;
		}

		#endregion

		#region Private Methods
		
        /// <summary>
		/// Executes the EPA engine.
		/// </summary>
		/// <param name="shape1">Shape1.</param>
		/// <param name="shape2">Shape2.</param>
		/// <param name="startPoint">Start point.</param>
		private EngineCollisionPoint ExecuteEngine(
			VertexProperties[] vertexShapeA,
			VertexProperties[] vertexShapeB,
			List<SupportTriangle> triangles,
			Vector3d centroid)
		{
			var epaCollisionPoint = new EngineCollisionPoint();

			double s = 0.0;
			double t = 0.0;

			var direction = new Vector3d ();
			var oldDirection = new Vector3d ();
			var vDistance = new Vector3d ();

			SupportTriangle epaBuffer;

			if (triangles.Count > 0) 
			{
				for (int k = 0; k < MaxIterations; k++) 
				{
					double minDistance = double.MaxValue;

					for (int i = 0; i < triangles.Count; i++) 
					{
						epaBuffer = triangles [i];

						if (!GeometryUtilities.TestCollinearity (
								epaBuffer.A.s,
								epaBuffer.B.s,
								epaBuffer.C.s)) 
						{
							vDistance = GeometryUtilities.GetPointTriangleIntersection (
								epaBuffer.A.s,
								epaBuffer.B.s,
								epaBuffer.C.s,
								origin,
								ref s,
								ref t).Value;
							
							epaBuffer.SetValueS (s);
							epaBuffer.SetValueT (t);
						} 
						else 
							continue;
							
						triangles [i] = epaBuffer;

						double distance = Vector3d.Length (vDistance);

						if (distance < minDistance) 
						{
							minDistance = distance;

							direction = vDistance;
							epaCollisionPoint.SetDist (vDistance);

                            epaCollisionPoint.SetNormal(triangles[i].Normal);

                            Helper.GetVertexFromMinkowsky(
								triangles[i],
								vertexShapeA,
								vertexShapeB,
								ref epaCollisionPoint);
						}
					}

					//L'origine risiede su uno dei bordi del triangolo
					if (Vector3d.Length(direction) < constTolerance)
					{
						direction = origin - centroid;
					}

					if (direction == oldDirection)
						break;

					Support vt = Helper.GetMinkowskiFarthestPoint(
							 vertexShapeA,
							 vertexShapeB,
							 direction.Normalize());

					triangles = Helper.AddPointToConvexPolygon (
						triangles,
						vt,
						centroid);

					oldDirection = direction;
				}
			}
			triangles.Clear ();

			return epaCollisionPoint;
		}

		#endregion

		#region Public Methods


		/// <summary>
		/// Execute Expanding Polytope Algorithm (EPA).
		/// </summary>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		/// <param name="startTriangles">Start triangles.</param>
		public EPAOutput Execute(
			VertexProperties[] vertexObjA,
			VertexProperties[] vertexObjB,
			List<SupportTriangle> startTriangles,
			Vector3d centroid)
		{
			EngineCollisionPoint epaCollisionPoint = ExecuteEngine (
													  vertexObjA,
													  vertexObjB,
													  startTriangles,
													  centroid);

            var distance = Vector3d.Length(epaCollisionPoint.Dist);

            var collisionPoint = new CollisionPoint (
				epaCollisionPoint.A,
				epaCollisionPoint.B,
				epaCollisionPoint.Normal,
                distance,
                true);

			return new EPAOutput (
				distance,
				collisionPoint);
		}


		#endregion
	}
}

