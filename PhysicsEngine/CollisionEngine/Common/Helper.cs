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
	internal static class Helper
	{
		#region Public Methods

		public static Support GetMinkowskiFarthestPoint(
			VertexProperties[] vertexObjA,
			VertexProperties[] vertexObjB,
			Vector3 direction)
		{
			int a = GetFarthestPoint(vertexObjA, direction);
			int b = GetFarthestPoint(vertexObjB, direction * -1.0);

			var sp = new Support(
								vertexObjA[a].Vertex - vertexObjB[b].Vertex,
								a,
								b);

			return sp;
		}

		public static int GetFarthestPoint(
			VertexProperties[] vertexObj,
			Vector3 direction)
		{
			if (vertexObj[0].Adjacency != null)
				return GetFarthestPointWithAdj(vertexObj, direction);
			else
				return GetFarthestPointWithOutAdj(vertexObj, direction);
		}

		public static VertexProperties GetVertexPosition(
			IGeometry obj,
			int vertexIndex)
		{
			return new VertexProperties(
				obj.Shape.Position +
				(obj.Shape.RotationMatrix * obj.RelativePosition[vertexIndex]),
				obj.VertexPosition[vertexIndex].Adjacency);
		}

		public static int GetFarthestPointWithOutAdj(
			VertexProperties[] vertexObj,
			Vector3 direction)
		{
			int index = 0;
			double maxDot = Vector3.Dot(vertexObj[index].Vertex, direction);

			for (int i = 1; i < vertexObj.Length; i++)
			{
				Vector3 vertex = vertexObj[i].Vertex;
				double dot = Vector3.Dot(vertex, direction);

				if (dot > maxDot)
				{
					maxDot = dot;
					index = i;
				}
			}
			return index;
		}

		public static int GetFarthestPointWithAdj(
			VertexProperties[] vertexObj,
			Vector3 direction)
		{
			int index = 0;
			bool check = true;
			double maxDot = Vector3.Dot(vertexObj[index].Vertex, direction);

			while (check)
			{
				check = false;
				int maxIndex = -1;

				for (int i = 0; i < vertexObj[index].Adjacency.Count; i++)
				{
					double dot = Vector3.Dot(vertexObj[vertexObj[index].Adjacency[i]].Vertex, direction);
					if (dot > maxDot)
					{
						maxDot = dot;
						maxIndex = vertexObj[index].Adjacency[i];
						check = true;
					}
				}

				if (maxIndex >= 0)
					index = maxIndex;
			}

			return index;
		}

		public static List<SupportTriangle> AddPointToConvexPolygon(
			List<SupportTriangle> triangles,
			Support vt,
			Vector3 centroid)
		{
			var result = new List<SupportTriangle>(triangles);

			var edges = new List<Edge>();
			int i = 0;

			while (i < result.Count)
			{
				Vector3 center = result[i].A.s;
				Vector3 dir = Vector3.Normalize(vt.s - center);

				if (Vector3.Dot(result[i].Normal, dir) > 0.0)
				{
					//Edge 1
					var edge = new Edge(
									result[i].A,
									result[i].B);

					edges = CheckEdge(edges, edge);

					//Edge 2

					edge = new Edge(
						result[i].A,
						result[i].C);

					edges = CheckEdge(edges, edge);

					//Edge 3

					edge = new Edge(
						result[i].B,
						result[i].C);

					edges = CheckEdge(edges, edge);

					result.RemoveAt(i);

				}
				else
				{
					i++;
				}
			}

			result = AddTriangle(edges, result, vt, centroid);
			edges = null;

			return result;
		}

		public static List<SupportTriangle> AddPointToConvexPolygon(
			List<SupportTriangle> triangles,
			List<Support> supportPoint,
			Support vt)
		{
			Vector3 centroid = FindPolygonCentroid(supportPoint);
			return AddPointToConvexPolygon(triangles, vt, centroid);
		}

		public static SupportTriangle TurnClockWiseNormal(
			SupportTriangle triangle,
			Vector3 v)
		{
			Vector3 centroidDiff = triangle.A.s - v;
			Vector3 normal = triangle.Normal;

			if (Vector3.Dot(triangle.Normal, centroidDiff) < 0.0)
				normal = triangle.Normal * -1.0;

			var tr = new SupportTriangle(
								triangle.A,
								triangle.B,
								triangle.C,
								triangle.S,
								triangle.T,
								normal);

			return tr;
		}

		public static Vector3 FindPolygonCentroid(List<Support> vertex)
		{
			var sum = new Vector3();

			for (int i = 0; i < vertex.Count; i++)
			{
				sum = sum + vertex[i].s;
			}

			return sum * (1.0 / vertex.Count);
		}

		public static Vector3 SetStartTriangle(
			ref List<SupportTriangle> triangles,
			Support[] startPoint)
		{
            var triangleSupport = new List<Support>
            {
                startPoint[0],
                startPoint[1],
                startPoint[2],
                startPoint[3]
            };

            //First triangle

            triangles.Add(
				AddTriangle(
				triangleSupport[0],
				triangleSupport[1],
				triangleSupport[2]));

			//Second triangle

			triangles.Add(
				AddTriangle(
				triangleSupport[0],
				triangleSupport[1],
				triangleSupport[3]));

			//Third triangle

			triangles.Add(
				AddTriangle(
				triangleSupport[0],
				triangleSupport[2],
				triangleSupport[3]));

			//Fourth triangle

			triangles.Add(
				AddTriangle(
				triangleSupport[1],
				triangleSupport[2],
				triangleSupport[3]));

			Vector3 centroid = FindPolygonCentroid(triangleSupport);

			triangles[0] = TurnClockWiseNormal(triangles[0], centroid);
			triangles[1] = TurnClockWiseNormal(triangles[1], centroid);
			triangles[2] = TurnClockWiseNormal(triangles[2], centroid);
			triangles[3] = TurnClockWiseNormal(triangles[3], centroid);

			triangleSupport.Clear();

			return centroid;
		}

		public static bool IsInConvexPoly(
			Vector3 p,
			List<SupportTriangle> triangles)
		{
			foreach (SupportTriangle spt in triangles)
			{
				Vector3 p2f = spt.A.s - p;         // f.v[0] is an arbitrary point on f
				double d = p2f.Dot(spt.Normal);
				d /= p2f.Length();                 // for numeric stability

				double bound = -1e-15; // use 1e15 to exclude boundaries
				if (d < bound)
					return false;
			}

			return true;
		}

		public static void GetVertexFromMinkowsky(
			SupportTriangle triangle,
			VertexProperties[] vertexShape1,
			VertexProperties[] vertexShape2,
			ref EngineCollisionPoint collisionPoint)
		{
			Vector3 a1 = vertexShape1[triangle.A.a].Vertex;
			Vector3 ba1 = vertexShape1[triangle.B.a].Vertex - a1;
			Vector3 ca1 = vertexShape1[triangle.C.a].Vertex - a1;

			Vector3 a2 = vertexShape2[triangle.A.b].Vertex;
			Vector3 ba2 = vertexShape2[triangle.B.b].Vertex - a2;
			Vector3 ca2 = vertexShape2[triangle.C.b].Vertex - a2;

			collisionPoint.SetA(
				new VertexProperties(a1 + (ba1 * triangle.S) + (ca1 * triangle.T),
				new int?[] { vertexShape1[triangle.A.a].ID, vertexShape1[triangle.B.a].ID, vertexShape1[triangle.C.a].ID }));
			collisionPoint.SetB(
				new VertexProperties(a2 + (ba2 * triangle.S) + (ca2 * triangle.T),
				new int?[] { vertexShape2[triangle.A.b].ID, vertexShape2[triangle.B.b].ID, vertexShape2[triangle.C.b].ID }));
		}

		public static VertexProperties[] SetVertexPosition(IGeometry obj)
		{
			VertexProperties[] vertexPosition = new VertexProperties[obj.VertexPosition.Length];

            for (int i = 0; i < obj.VertexPosition.Length; i++)
            {
                vertexPosition[i] = GetVertexPosition(obj, i);
            }

			return vertexPosition;
		}

		#endregion

		#region Private Methods

		private static List<SupportTriangle> AddTriangle(
			List<Edge> edge,
			List<SupportTriangle> triangles,
			Support p,
			Vector3 centroid)
		{
			var result = new List<SupportTriangle>(triangles);

			for (int i = 0; i < edge.Count; i++)
			{
				var tri = new SupportTriangle(
									  edge[i].A,
									  edge[i].B,
									  p,
									  0.0,
									  0.0,
									  GeometryUtilities.CalculateNormal(edge[i].A.s, edge[i].B.s, p.s));

				tri = TurnClockWiseNormal(tri, centroid);
				result.Add(tri);
			}

			return result;
		}

		private static List<Edge> CheckEdge(
			List<Edge> edge, 
			Edge ed)
		{
			var result = new List<Edge>(edge);

			bool test = false;
			int i = 0;
			while (i < result.Count)
			{
				if (result[i].A.s == ed.A.s &&
					result[i].B.s == ed.B.s)
				{
					test = true;
					result.RemoveAt(i);
				}
				else
					i++;
			}
			if (!test)
				result.Add(ed);

			return result;
		}

		private static SupportTriangle AddTriangle(
			Support a,
			Support b,
			Support c)
		{
			var epaTriangle = new SupportTriangle(
				a,
				b,
				c,
				0.0,
				0.0,
				GeometryUtilities.CalculateNormal(
					a.s,
					b.s,
					c.s));

			return epaTriangle;
		}

		#endregion 
	}
}
