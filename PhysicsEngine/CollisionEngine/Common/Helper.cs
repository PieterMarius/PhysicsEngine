using System.Collections.Generic;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	public static class Helper
	{
		#region Public Methods

		public static Support GetMinkowskiFarthestPoint(
            IGeometry objA,
            IGeometry objB,
            Vector3[] vertexObjA,
            Vector3[] vertexObjB,
            Vector3 direction)
		{
            int a = 0;
            int b = 0;
            if (objA.GeometryType == ObjectGeometryType.ConvexBody)
                a = GetFarthestPoint(objA, vertexObjA, direction);
            else
                a = GetNonConvexFarthestPoint(objA, vertexObjA, direction);

            if (objB.GeometryType == ObjectGeometryType.ConvexBody)
                b = GetFarthestPoint(objB, vertexObjB, direction * -1.0);
            else
                b = GetNonConvexFarthestPoint(objB, vertexObjB, direction * -1.0);
            
            	var sp = new Support(
                vertexObjA[a] - vertexObjB[b],
				a,
				b);

			return sp;
		}

		public static int GetFarthestPoint(
			IGeometry obj,
            Vector3[] vertexObj,
			Vector3 direction)
		{
			int index = 0;
            bool check = true;
            double maxDot = Vector3.Dot(vertexObj[index], direction);
            
            while(check)
            {
                check = false;
                int maxIndex = -1;

                for (int i = 0; i < obj.VertexPosition[index].Adjacency.Count; i++)
                {
                    double dot = Vector3.Dot(vertexObj[obj.VertexPosition[index].Adjacency[i]], direction);
                    if (dot > maxDot)
                    {
                        maxDot = dot;
                        maxIndex = obj.VertexPosition[index].Adjacency[i];
                        check = true;
                    }
                }

                if(maxIndex >= 0)
                    index = maxIndex;
            }

            return index;
        }

        public static Vector3 GetVertexPosition(
            IGeometry obj,
            int vertexIndex)
        {
            return
                obj.Shape.Position +
                (obj.Shape.RotationMatrix * obj.RelativePosition[vertexIndex]);
        }

        public static int GetNonConvexFarthestPoint(
            IGeometry obj,
            Vector3[] vertexObj,
            Vector3 direction)
        {
            int index = 0;
            double maxDot = Vector3.Dot(vertexObj[index], direction);

            for (int i = 1; i < obj.VertexPosition.Length; i++)
            {
                Vector3 vertex = vertexObj[i];
                double dot = Vector3.Dot(vertex, direction);

                if (dot > maxDot)
                {
                    maxDot = dot;
                    index = i;
                }
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
				Vector3 center = result[i].a.s;
				Vector3 dir = Vector3.Normalize(vt.s - center);

				if (Vector3.Dot(result[i].normal, dir) > 0.0)
				{
					//Edge 1
					var edge = new Edge(
									result[i].a,
									result[i].b);

					edges = CheckEdge(edges, edge);

					//Edge 2

					edge = new Edge(
						result[i].a,
						result[i].c);

					edges = CheckEdge(edges, edge);

					//Edge 3

					edge = new Edge(
						result[i].b,
						result[i].c);

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
			Vector3 centroidDiff = triangle.a.s - v;
			Vector3 normal = triangle.normal;

			if (Vector3.Dot(triangle.normal, centroidDiff) < 0.0)
				normal = triangle.normal * -1.0;

			var tr = new SupportTriangle(
								triangle.a,
								triangle.b,
								triangle.c,
								triangle.s,
								triangle.t,
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
			var triangleSupport = new List<Support>();
			triangleSupport.Add(startPoint[0]);
			triangleSupport.Add(startPoint[1]);
			triangleSupport.Add(startPoint[2]);
			triangleSupport.Add(startPoint[3]);

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
                Vector3 p2f = spt.a.s - p;         // f.v[0] is an arbitrary point on f
                double d = p2f.Dot(spt.normal);
                d /= p2f.Length();                 // for numeric stability

                double bound = -1e-15; // use 1e15 to exclude boundaries
                if (d < bound)
                    return false;
            }

            return true;
        }

		public static void GetVertexFromMinkowsky(
			SupportTriangle triangle,
            Vector3[] vertexShape1,
            Vector3[] vertexShape2,
            ref EngineCollisionPoint collisionPoint)
		{
			Vector3 a1 = vertexShape1[triangle.a.a];
			Vector3 ba1 = vertexShape1[triangle.b.a] - a1;
			Vector3 ca1 = vertexShape1[triangle.c.a] - a1;

			Vector3 a2 = vertexShape2[triangle.a.b];
            Vector3 ba2 = vertexShape2[triangle.b.b] - a2;
			Vector3 ca2 = vertexShape2[triangle.c.b] - a2;

			collisionPoint.SetA(a1 + (ba1 * triangle.s) + (ca1 * triangle.t));
			collisionPoint.SetB(a2 + (ba2 * triangle.s) + (ca2 * triangle.t));
		}

        public static Vector3[] SetVertexPosition(IGeometry obj)
        {
            Vector3[] vertexPosition = new Vector3[obj.VertexPosition.Length];

            for (int i = 0; i < obj.VertexPosition.Length; i++)
                vertexPosition[i] = GetVertexPosition(obj, i);

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
									  edge[i].a,
									  edge[i].b,
									  p,
									  0.0,
									  0.0,
									  GeometryUtilities.CalculateNormal(edge[i].a.s, edge[i].b.s, p.s));

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
				if (result[i].a.s == ed.a.s &&
					result[i].b.s == ed.b.s)
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
