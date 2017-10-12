using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;


namespace SharpPhysicsEngine.CollisionEngine
{
	public sealed class EPA
	{
		#region Settings Variable

		public int MaxIterations { get; private set; }
		public double Precision { get; private set; }
		public double EPAManifoldTolerance { get; private set;}
		public int ManifoldPointNumber { get; private set;}

		readonly Vector3 origin = new Vector3();

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
						   
		//private void GetEpaVertexFromMinkowsky(
		//	SupportTriangle triangle,
		//	VertexProperties[] vertexShape1,
		//	VertexProperties[] vertexShape2,
		//	ref EngineCollisionPoint epaCollisionPoint)
		//{
		//	Vector3 a1 = vertexShape1[triangle.a.a].Vertex;
		//	Vector3 ba1 = vertexShape1[triangle.b.a].Vertex - a1;
		//	Vector3 ca1 = vertexShape1[triangle.c.a].Vertex - a1;

		//	Vector3 a2 = vertexShape2[triangle.a.b].Vertex;
		//	Vector3 ba2 = vertexShape2[triangle.b.b].Vertex - a2;
		//	Vector3 ca2 = vertexShape2[triangle.c.b].Vertex - a2;

		//	epaCollisionPoint.SetA (a1 + (ba1 * triangle.s) + (ca1 * triangle.t));
		//	epaCollisionPoint.SetB (a2 + (ba2 * triangle.s) + (ca2 * triangle.t));
		//}

		/// <summary>
		/// Executes the EPA engine.
		/// </summary>
		/// <param name="shape1">Shape1.</param>
		/// <param name="shape2">Shape2.</param>
		/// <param name="startPoint">Start point.</param>
		private EngineCollisionPoint ExecuteEngine(
			VertexProperties[] vertexShape1,
			VertexProperties[] vertexShape2,
			List<SupportTriangle> triangles,
			Vector3 centroid)
		{
			var epaCollisionPoint = new EngineCollisionPoint();

			double s = 0.0;
			double t = 0.0;

			var direction = new Vector3 ();
			var oldDirection = new Vector3 ();
			var vDistance = new Vector3 ();

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
								epaBuffer.a.s,
								epaBuffer.b.s,
								epaBuffer.c.s)) 
						{
							vDistance = GeometryUtilities.GetPointTriangleIntersection (
								epaBuffer.a.s,
								epaBuffer.b.s,
								epaBuffer.c.s,
								origin,
								ref s,
								ref t).Value;
							
							epaBuffer.SetValueS (s);
							epaBuffer.SetValueT (t);
						} 
						else 
						{
							continue;
						}
							
						triangles [i] = epaBuffer;

						double distance = Vector3.Length (vDistance);

						if (distance < minDistance) 
						{
							minDistance = distance;

							direction = vDistance;
							epaCollisionPoint.SetDist (vDistance);
							epaCollisionPoint.SetNormal (Vector3.Normalize (vDistance));

							Helper.GetVertexFromMinkowsky(
								triangles[i],
								vertexShape1,
								vertexShape2,
								ref epaCollisionPoint);
						}
					}

					//L'origine risiede su uno dei bordi del triangolo
					if (Vector3.Length(direction) < constTolerance)
					{
						direction = origin - centroid;
					}

					if (direction == oldDirection)
						break;

					Support vt = Helper.GetMinkowskiFarthestPoint(
							 vertexShape1,
							 vertexShape2,
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
			Vector3 centroid)
		{
			EngineCollisionPoint epaCollisionPoint = ExecuteEngine (
													  vertexObjA,
													  vertexObjB,
													  startTriangles,
													  centroid);

			var collisionPoint = new CollisionPoint (
				epaCollisionPoint.a,
				epaCollisionPoint.b,
				epaCollisionPoint.normal);

			return new EPAOutput (
				Vector3.Length (epaCollisionPoint.dist),
				collisionPoint);
		}


		#endregion
	}
}

