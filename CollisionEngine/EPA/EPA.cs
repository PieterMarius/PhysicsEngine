using System;
using System.Collections.Generic;
using System.Diagnostics;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;


namespace CollisionEngine
{
	public class EPA
	{
		#region Settings Variable

		public int MaxIterations { get; private set; }
		public double Precision { get; private set; }
		public double EPAManifoldTolerance { get; private set;}
		public int ManifoldPointNumber { get; private set;}

		private Vector3 origin;

		#endregion

		#region Constructor

		public EPA (
			int maxIterations,
			double precision,
			double epaTolerance,
			int manifoldPointNumber)
		{
			this.MaxIterations = maxIterations;
			this.Precision = precision;
			this.EPAManifoldTolerance = epaTolerance;
			this.ManifoldPointNumber = manifoldPointNumber;

			this.origin = new Vector3 (0.0, 0.0, 0.0);
		}

		#endregion

		#region Private Methods

		private Vector3 findPolygonCentroid(List<Support> vertex) {
			Vector3 sum = new Vector3 ();
			for (int i = 0; i < vertex.Count; i++) {
				sum = sum + vertex [i].s;
			}

			return sum * (1.0 / vertex.Count);
		}
			
		private Vector3 findTriangleCentroid(EpaTriangle triangle)
		{
			Vector3 result = (triangle.a.s + triangle.b.s + triangle.c.s) * (1.0 / 3.0);
			return result;
		}

		private EpaTriangle turnClockWiseNormal(
			EpaTriangle triangle,
			Vector3 v)
		{
			Vector3 centridDiff = triangle.a.s - v;
			Vector3 normal = triangle.normal;

			if (Vector3.Dot (triangle.normal, centridDiff) < 0.0)
				normal = triangle.normal * -1.0;

			EpaTriangle tr = new EpaTriangle (
				                triangle.a,
				                triangle.b,
				                triangle.c,
				                triangle.s,
				                triangle.t,
				                normal);

			return tr;
		}

		private int getFarthestPoint(
			ObjectGeometry obj, 
			Vector3 direction)
		{
			int index = 0;
			double maxDot = Vector3.Dot (obj.VertexPosition [index], direction);

			for (int i = 1; i < obj.VertexPosition.Length; i++) 
			{
				double dot = Vector3.Dot (obj.VertexPosition [i], direction);

				if (dot > maxDot) 
				{
					maxDot = dot;
					index = i;
				}
			}
			return index;
		}

		private Support getMinkowskiFarthestPoint(
			ObjectGeometry obj1, 
			ObjectGeometry obj2,
			Vector3 direction)
		{

			int a = getFarthestPoint (obj1, direction);
			int b = getFarthestPoint (obj2, direction * -1.0);
			Support sp = new Support (
				obj1.VertexPosition [a] - obj2.VertexPosition [b],
				a,
				b);

			return sp;
		}

		private bool testTetrahedronAlignedPoint(List<Support> point)
		{
			Vector3 a = point[0].s;
			Vector3 b = point[1].s;
			Vector3 c = point[2].s;
			Vector3 d = point[3].s;

			double[] t = new double[24];

			t[0] = c.x * b.y * a.z;
			t[1] = d.x * b.y * a.z;
			t[2] = b.x * c.y * a.z;
			t[3] = d.x * c.y * a.z;
			t[4] = b.x * d.y * a.z;
			t[5] = c.x * d.y * a.z;

			t[6] = c.x * a.y * b.z;
			t[7] = d.x * a.y * b.z;
			t[8] = a.x * c.y * b.z;
			t[9] = d.x * c.y * b.z;
			t[10] = a.x * d.y * b.z;
			t[11] = c.x * d.y * b.z;

			t[12] = b.x * a.y * c.z;
			t[13] = d.x * a.y * c.z;
			t[14] = a.x * b.y * c.z;
			t[15] = d.x * b.y * c.z;
			t[16] = a.x * d.y * c.z;
			t[17] = b.x * d.y * c.z;

			t[18] = b.x * a.y * d.z;
			t[19] = c.x * a.y * d.z;
			t[20] = a.x * b.y * d.z;
			t[21] = c.x * b.y * d.z;
			t[22] = a.x * c.y * d.z;
			t[23] = b.x * c.y * d.z;

			double d0 = -t[0] + t[1] + t[2] - t[3] - t[4] + t[5] +
				t[6] - t[7] - t[8] + t[9] + t[10] - t[11] -
				t[12] + t[13] + t[14] - t[15] - t[16] + t[17]+
				t[18] - t[19] - t[20] + t[21] + t[22] - t[23];

			//Test if points are aligned or not
			if (Math.Abs(d0) < ConstValues.precision) 
				return true;

			return false;
		}
			
		private Vector3 startTriangle(
			ref List<EpaTriangle> triangles,
			Support[] startPoint)
		{
			List<Support> triangleSupport = new List<Support> ();
			triangleSupport.Add (startPoint [0]);
			triangleSupport.Add (startPoint [1]);
			triangleSupport.Add (startPoint [2]);
			triangleSupport.Add (startPoint [3]);

			//First triangle

			triangles.Add (
				this.addTriangle (
				triangleSupport [0],
				triangleSupport [1],
				triangleSupport [2]));

			//Second triangle

			triangles.Add (
				this.addTriangle (
				triangleSupport [0],
				triangleSupport [1],
				triangleSupport [3]));

			//Third triangle

			triangles.Add (
				this.addTriangle (
				triangleSupport [0],
				triangleSupport [2],
				triangleSupport [3]));
			
			//Fourth triangle

			triangles.Add (
				this.addTriangle (
				triangleSupport [1],
				triangleSupport [2],
				triangleSupport [3]));

			Vector3 centroid = this.findPolygonCentroid (triangleSupport);

			triangles [0] = this.turnClockWiseNormal (triangles [0], centroid);
			triangles [1] = this.turnClockWiseNormal (triangles [1], centroid);
			triangles [2] = this.turnClockWiseNormal (triangles [2], centroid);
			triangles [3] = this.turnClockWiseNormal (triangles [3], centroid);

			triangleSupport.Clear ();

			return centroid;
		}

		private EpaTriangle addTriangle(
			Support a,
			Support b,
			Support c)
		{
			EpaTriangle epaTriangle = new EpaTriangle(
				a,
				b,
				c,
				0.0,
				0.0,
				GeometryUtilities.CalculateNormal (
					a.s,
					b.s,
					c.s));

			return epaTriangle;
		}
			


		//Add triangles to convex hull
		private void addTriangle(
			List<Edge> edge,
			ref List<EpaTriangle> triangles,
			Support p,
			Vector3 centroid)
		{
			for (int i = 0; i < edge.Count; i++) 
			{
				 
				EpaTriangle tri = new EpaTriangle (
					                  edge [i].a,
					                  edge [i].b,
					                  p,
					                  0.0,
					                  0.0,
					                  GeometryUtilities.CalculateNormal (edge [i].a.s, edge [i].b.s, p.s));
				
				tri = turnClockWiseNormal (tri, centroid);
				triangles.Add (tri);
			}
		}

		private void checkEdge(ref List<Edge> edge, Edge ed)
		{
			bool test = false;
			int i = 0;
			while (i < edge.Count) 
			{
				if (edge [i].a.s == ed.a.s &&
				    edge [i].b.s == ed.b.s) {
					test = true;
					edge.RemoveAt (i);
				} else
					i++;
			}
			if (!test)
				edge.Add (ed);
		}

		private void addPointToConvexPolygon(
			ObjectGeometry shape1,
			ObjectGeometry shape2,
			Vector3 centroid,
			ref List<EpaTriangle> triangles,
			Vector3 direction)
		{
			Support vt = this.getMinkowskiFarthestPoint (
				             shape1,
				             shape2,
				             direction);
			
			List<Edge> edges = new List<Edge> ();
			int i = 0;
			while (i < triangles.Count) 
			{
				Vector3 center = triangles [i].a.s;
				Vector3 dir = Vector3.Normalize (vt.s - center);

				if (Vector3.Dot (triangles [i].normal, dir) > 0.0) 
				{
					//Edge 1

					Edge edge = new Edge (
						            triangles [i].a,
						            triangles [i].b);

					this.checkEdge (ref edges, edge);

					//Edge 2

					edge = new Edge (
						triangles [i].a,
						triangles [i].c);

					this.checkEdge (ref edges, edge);

					//Edge 3

					edge = new Edge (
						triangles [i].b,
						triangles [i].c);

					this.checkEdge (ref edges, edge);

					triangles.RemoveAt (i);

				} 
				else 
				{
					i++;
				}
			}

			this.addTriangle (edges, ref triangles, vt, centroid);
			edges.Clear ();
		}

		private void getEpaVertexFromMinkowsky(
			EpaTriangle triangle,
			ObjectGeometry shape1,
			ObjectGeometry shape2,
			ref EpaCollisionPoint epaCollisionPoint)
		{
			Vector3 a1 = shape1.VertexPosition [triangle.a.a];
			Vector3 ba1 = shape1.VertexPosition [triangle.b.a] - a1;
			Vector3 ca1 = shape1.VertexPosition [triangle.c.a] - a1;

			Vector3 a2 = shape2.VertexPosition [triangle.a.b];
			Vector3 ba2 = shape2.VertexPosition [triangle.b.b] - a2;
			Vector3 ca2 = shape2.VertexPosition [triangle.c.b] - a2;
			epaCollisionPoint.SetA (a1 + (ba1 * triangle.s) + (ca1 * triangle.t));
			epaCollisionPoint.SetB (a2 + (ba2 * triangle.s) + (ca2 * triangle.t));
		}

		private Vector3 getRandomDirection()
		{
			return Vector3.Normalize (new Vector3 (
				GeometryUtilities.GetRandom (-1.0, 1.0), 
				GeometryUtilities.GetRandom (-1.0, 1.0), 
				GeometryUtilities.GetRandom (-1.0, 1.0)));
		}
	
		/// <summary>
		/// Executes the EPA engine.
		/// </summary>
		/// <param name="shape1">Shape1.</param>
		/// <param name="shape2">Shape2.</param>
		/// <param name="startPoint">Start point.</param>
		/// <param name="tri">Tri.</param>
		/// <param name="epaCollisionPoint">Epa collision point.</param>
		/// <param name="supportList">Support list.</param>
		private EpaCollisionPoint executeEPAEngine(
			ObjectGeometry shape1,
			ObjectGeometry shape2,
			Support[] startPoint)
		{

			EpaCollisionPoint epaCollisionPoint = new EpaCollisionPoint (
				new Vector3 (),
				new Vector3 (),
				new Vector3 (),
				new Vector3 ());

			double s = 0.0;
			double t = 0.0;

			List<EpaTriangle> triangles = new List<EpaTriangle> ();
			Vector3 centroid = this.startTriangle (
				                   ref triangles,
				                   startPoint);

			Vector3 direction = new Vector3 ();
			Vector3 oldDirection = new Vector3 ();
			Vector3 vDistance = new Vector3 ();
			EpaTriangle epaBuffer;

			if (triangles.Count > 0) 
			{
				for (int k = 0; k < this.MaxIterations; k++) 
				{
					double minDistance = double.MaxValue;

					for (int i = 0; i < triangles.Count; i++) 
					{
						epaBuffer = triangles [i];

						if (!GeometryUtilities.TestCollinearity (
							    epaBuffer.a.s,
							    epaBuffer.b.s,
							    epaBuffer.c.s)) {

							vDistance = GeometryUtilities.GetPointTriangleIntersection (
								epaBuffer.a.s,
								epaBuffer.b.s,
								epaBuffer.c.s,
								this.origin,
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

							this.getEpaVertexFromMinkowsky (
								triangles [i],
								shape1,
								shape2,
								ref epaCollisionPoint);
						}
					}

					if (Vector3.Length (direction) < ConstValues.precision) 
					{
						direction = this.getRandomDirection ();	
					}
						
					this.addPointToConvexPolygon (
						shape1,
						shape2,
						centroid,
						ref triangles,
						Vector3.Normalize (direction));

					//Early exit
					if (direction == oldDirection)
						break;

					oldDirection = direction;
				}
			}
			triangles.Clear ();

			return epaCollisionPoint;
		}

		#endregion

		#region Public Methods

		public EPAOutput GetCompenetrationDistance(
			ObjectGeometry objectA,
			ObjectGeometry objectB,
			Support[] startTriangles)
		{

			EpaCollisionPoint epaCollisionPoint = this.executeEPAEngine (
				                                      objectA,
				                                      objectB,
				                                      startTriangles);

			CollisionPoint collisionPoint = new CollisionPoint (
				epaCollisionPoint.a,
				epaCollisionPoint.b,
				epaCollisionPoint.normal);

			return  new EPAOutput (
				Vector3.Length (epaCollisionPoint.dist),
				collisionPoint);
		}


		#endregion
	}
}

