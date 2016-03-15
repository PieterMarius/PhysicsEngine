using System;
using System.Collections.Generic;
using System.Linq;

namespace PhysicsEngineMathUtility
{
	public static class GeometryUtilities
	{
		#region Private Static Methods

		private static readonly Random random = new Random();
		private static readonly object syncLock = new object();

		#endregion

		#region Public Static Methods


		/// <summary>
		/// Gets the projected point on line.
		/// </summary>
		/// <returns>The projected point on line.</returns>
		/// <param name="a">The alpha component.</param>
		/// <param name="b">The blue component.</param>
		/// <param name="p">P.</param>
		/// <param name="t">T.</param>
		public static Vector3 GetProjectedPointOnLine(
			Vector3 a, 
			Vector3 b, 
			Vector3 p,
			ref double  t) 
		{
			Vector3 c = b - a;
			Vector3 d = p - a;
			double mod = Vector3.Length(c);
			t = Vector3.Dot(c, d) / (mod * mod);

			if (t < 0.0) 
				return a;
			if (t > 1.0) 
				return b;

			return (a + ( c * t ));
		}

		/// <summary>
		/// Tests the lines intersect.
		/// </summary>
		/// <returns><c>true</c>, if lines intersect was tested, <c>false</c> otherwise.</returns>
		/// <param name="p1">P1.</param>
		/// <param name="p2">P2.</param>
		/// <param name="p3">P3.</param>
		/// <param name="p4">P4.</param>
		/// <param name="pa">Pa.</param>
		/// <param name="pb">Pb.</param>
		/// <param name="mua">Mua.</param>
		/// <param name="mub">Mub.</param>
		public static bool TestLinesIntersect(
			Vector3 p1, 
			Vector3 p2, 
			Vector3 p3, 
			Vector3 p4, 
			ref Vector3 pa, 
			ref Vector3 pb, 
			ref double mua, 
			ref double mub) 
		{
			Vector3 p13, p43, p21;
			double d1343, d4321, d1321, d4343, d2121;
			double numer, denom;

			p13 = p1 - p3;
			p43 = p4 - p3;

			if (Math.Abs(p43.x) < ConstValues.precision && 
				Math.Abs(p43.y) < ConstValues.precision && 
				Math.Abs(p43.z) < ConstValues.precision) 
				return false;

			p21 = p2 - p1;

			if (Math.Abs(p21.x) < ConstValues.precision && 
				Math.Abs(p21.y) < ConstValues.precision && 
				Math.Abs(p21.z) < ConstValues.precision) 
				return false;
			
			d1343 = Vector3.Dot(p13, p43);
			d4321 = Vector3.Dot(p43, p21);
			d1321 = Vector3.Dot(p13, p21);
			d4343 = Vector3.Dot(p43, p43);
			d2121 = Vector3.Dot(p21, p21);

			denom = (d2121 * d4343) - (d4321 * d4321);
			if (Math.Abs(denom) < ConstValues.precision) 
				return false;

			numer = d1343 * d4321 - d1321 * d4343;

			mua = numer / denom;
			mub = (d1343 + d4321 * mua) / d4343;

			pa = p1 + (p21 * mua);
			pb = p3 + (p43 * mub);

			return true;
		}

		/// <summary>
		/// Calculates normal.
		/// </summary>
		/// <returns>The normal.</returns>
		/// <param name="a">The alpha component.</param>
		/// <param name="b">The blue component.</param>
		/// <param name="c">C.</param>
		public static Vector3 CalculateNormal(
			Vector3 a,
			Vector3 b,
			Vector3 c)
		{
			Vector3 v1 = b - a;
			Vector3 v2 = c - a;

			double x = (v1.y * v2.z) - (v1.z * v2.y);
			double y = -((v2.z * v1.x) - (v2.x * v1.z));
			double z = (v1.x * v2.y) - (v1.y * v2.x);

			return  Vector3.Normalize(new Vector3(x, y,z));
		}
			
		/// <summary>
		/// Projects the vector on plane.
		/// </summary>
		/// <returns>The vector on plane.</returns>
		/// <param name="normal">Normal.</param>
		public static Vector3 ProjectVectorOnPlane(Vector3 normal)
		{
			if (Math.Abs (normal.x) < 0.000000001) 
				return new Vector3 (1.0, 0.0, 0.0);
				
			if (Math.Abs (normal.y) < 0.000000001) 
				return new Vector3 (0.0, 1.0, 0.0);
				
			if (Math.Abs (normal.z) < 0.000000001) 
				return new Vector3 (0.0, 0.0, 1.0);

			double z = -(normal.x + normal.y) / normal.z;

			return Vector3.Normalize(new Vector3 (1.0, 1.0, z));
		}
			
		/// <summary>
		/// Gets random double.
		/// </summary>
		/// <returns>The random.</returns>
		/// <param name="min">Minimum.</param>
		/// <param name="max">Max.</param>
		public static double GetRandom(double min, double max)
		{
			lock (syncLock) {
				return random.NextDouble () * (max - min) + min;
			}
		}

		/// <summary>
		/// Gets random int.
		/// </summary>
		/// <returns>The random.</returns>
		/// <param name="min">Minimum.</param>
		/// <param name="max">Max.</param>
		public static int GetRandom(int min, int max)
		{
			lock (syncLock) {
				return random.Next (min, max);
			}
		}

		/// <summary>
		/// Clamp the specified v, max and min.
		/// </summary>
		/// <param name="v">V.</param>
		/// <param name="max">Max.</param>
		/// <param name="min">Minimum.</param>
		public static double Clamp(
			double v,
			double max,
			double min)
		{
			return (v > max) ? max : ((v < min) ? min : v);
		}
			
		/// <summary>
		/// Gets the point triangle intersection, return null if it can't find correct solution.
		/// </summary>
		/// <returns>The point triangle intersection.</returns>
		/// <param name="v1">V1.</param>
		/// <param name="v2">V2.</param>
		/// <param name="v3">V3.</param>
		/// <param name="p">P.</param>
		/// <param name="t0">T0.</param>
		/// <param name="t1">T1.</param>
		public static Vector3? GetPointTriangleIntersection(
			Vector3 v1,
			Vector3 v2,
			Vector3 v3,
			Vector3 p,
			ref double t0,
			ref double t1)
		{
			Vector3 diff = p - v1;
			Vector3 edge0 = v2 - v1;
			Vector3 edge1 = v3 - v1;
			double a00 = Vector3.Dot(edge0, edge0);
			double a01 = Vector3.Dot(edge0, edge1);
			double a11 = Vector3.Dot(edge1, edge1);
			double b0 = -Vector3.Dot(diff, edge0);
			double b1 = -Vector3.Dot(diff, edge1);
			double det = a00 * a11 - a01 * a01;
			t0 = a01 * b1 - a11 * b0;
			t1 = a01 * b0 - a00 * b1;

			if (t0 + t1 <= det)
			{
				if (t0 < 0.0)
				{
					if (t1 < 0.0)  // region 4
					{
						if (b0 < 0.0)
						{
							t1 = 0.0;
							if (-b0 >= a00)  // V0
							{
								t0 = 1.0;
							}
							else  // E01
							{
								t0 = -b0 / a00;
							}
						}
						else
						{
							t0 = 0.0;
							if (b1 >= 0.0)  // V0
							{
								t1 = 0.0;
							}
							else if (-b1 >= a11)  // V2
							{
								t1 = 1.0;
							}
							else  // E20
							{
								t1 = -b1 / a11;
							}
						}
					}
					else  // region 3
					{
						t0 = 0.0;
						if (b1 >= 0.0)  // V0
						{
							t1 = 0.0;
						}
						else if (-b1 >= a11)  // V2
						{
							t1 = 1.0;
						}
						else  // E20
						{
							t1 = -b1 / a11;
						}
					}
				}
				else if (t1 < 0.0)  // region 5
				{
					t1 = 0.0;
					if (b0 >= 0.0)  // V0
					{
						t0 = 0.0;
					}
					else if (-b0 >= a00)  // V1
					{
						t0 = 1.0;
					}
					else  // E01
					{
						t0 = -b0 / a00;
					}
				}
				else  // region 0, interior
				{
					if (det != 0.0) {
						double invDet = 1.0 / det;
						t0 *= invDet;
						t1 *= invDet;

					} else {
						return null;
					}
				}
			}
			else
			{
				double tmp0, tmp1, numer, denom;

				if (t0 < 0.0)  // region 2
				{
					tmp0 = a01 + b0;
					tmp1 = a11 + b1;
					if (tmp1 > tmp0)
					{
						numer = tmp1 - tmp0;
						denom = a00 - 2.0 * a01 + a11;
						if (numer >= denom)  // V1
						{
							t0 = 1.0;
							t1 = 0.0;
						}
						else  // E12
						{
							t0 = numer / denom;
							t1 = 1.0 - t0;
						}
					}
					else
					{
						t0 = 0.0;
						if (tmp1 <= 0.0)  // V2
						{
							t1 = 1.0;
						}
						else if (b1 >= 0.0)  // V0
						{
							t1 = 0.0;
						}
						else  // E20
						{
							t1 = -b1 / a11;
						}
					}
				}
				else if (t1 < 0.0)  // region 6
				{
					tmp0 = a01 + b1;
					tmp1 = a00 + b0;
					if (tmp1 > tmp0)
					{
						numer = tmp1 - tmp0;
						denom = a00 - 2.0 * a01 + a11;
						if (numer >= denom)  // V2
						{
							t1 = 1.0;
							t0 = 0.0;
						}
						else  // E12
						{
							t1 = numer / denom;
							t0 = 1.0 - t1;
						}
					}
					else
					{
						t1 = 0.0;
						if (tmp1 <= 0.0)  // V1
						{
							t0 = 1.0;
						}
						else if (b0 >= 0.0)  // V0
						{
							t0 = 0.0;
						}
						else  // E01
						{
							t0 = -b0 / a00;
						}
					}
				}
				else  // region 1
				{
					numer = a11 + b1 - a01 - b0;
					if (numer <= 0.0)  // V2
					{
						t0 = 0.0;
						t1 = 1.0;
					}
					else
					{
						denom = a00 - 2.0 * a01 + a11;
						if (numer >= denom)  // V1
						{
							t0 = 1.0;
							t1 = 0.0;
						}
						else  // 12
						{
							t0 = numer / denom;
							t1 = 1.0 - t0;
						}
					}
				}
			}

			Vector3 result = v1 + t0 * edge0 + t1 * edge1;
			return result;
		}

		/// <summary>
		/// Tests collinearity.
		/// </summary>
		/// <returns><c>true</c>, if collinearity was tested, <c>false</c> otherwise.</returns>
		/// <param name="a">The alpha component.</param>
		/// <param name="b">The blue component.</param>
		/// <param name="c">C.</param>
		public static bool TestCollinearity(
			Vector3 a,
			Vector3 b,
			Vector3 c)
		{
			Vector3 d = b - a;
			Vector3 e = c - a;

			Vector3 f = Vector3.Cross (d, e);
			if (Math.Abs (f.x) < 0.000001 &&
			    Math.Abs (f.y) < 0.000001 &&
			    Math.Abs (f.z) < 0.000001)
				return true;

			return false;
		}

		/// <summary>
		/// Gets barycentric point.
		/// </summary>
		/// <returns>The barycentric point.</returns>
		/// <param name="vertex">Vertex.</param>
		public static Vector3 GetBarycentricPoint(Vector3[] vertex)
		{
			Vector3 t = new Vector3 ();
			for (int i = 0; i < vertex.Length; i++) {
				t = t + vertex [i];
			}

			return t * (1.0 / vertex.Length);
		}

		/// <summary>
		/// Calculates angle.
		/// </summary>
		/// <returns>The angle.</returns>
		/// <param name="a">The alpha component.</param>
		/// <param name="b">The blue component.</param>
		/// <param name="normal">Normal.</param>
		public static double CalculateAngle(
			Vector3 a,
			Vector3 b,
			Vector3 normal)
		{
			Vector3 c = Vector3.Cross (a, b);

			double angle = Math.Atan2 (Vector3.Length (c), Vector3.Dot (a, b));

			return Vector3.Dot (c, normal) < 0.0 ? -angle : angle; 

		}

		/// <summary>
		/// Turns vector clock wise.
		/// </summary>
		/// <returns>The vector clock wise.</returns>
		/// <param name="vertices">Vertices.</param>
		/// <param name="normal">Normal.</param>
		public static Vector3[] TurnVectorClockWise(
			Vector3[] vertices,
			Vector3 normal)
		{
			if (vertices.Length > 0) {
				Vector3 barycentricPoint = GetBarycentricPoint (vertices);

				Vertices[] vert = new Vertices[vertices.Length];

				vert [0] = new Vertices (vertices [0], 0.0);

				for (int i = 1; i < vertices.Length; ++i) 
				{
					vert [i] = new Vertices (
						vertices [i],
						CalculateAngle (
							vertices [0] - barycentricPoint, 
							vertices [i] - barycentricPoint, 
							normal));
				}

				Array.Sort (vert);

				Vector3[] result = new Vector3[vert.Length];

				for (int i = 0; i < vert.Length; i++) 
				{
					result [i] = vert [i].Vertex;
				}

				return result;
			} 
			else 
			{
				throw new Exception ("Wrong number of vertices.");	
			}
		}

		/// <summary>
		/// Tests if point inside polygon.
		/// </summary>
		/// <returns>The point inside polygon.</returns>
		/// <param name="vertex">Vertex.</param>
		/// <param name="point">Point.</param>
		/// <param name="planeNormal">Plane normal.</param>
		/// <param name="referencePoint">Reference point.</param>
		public static double TestPointInsidePolygon(
			Vector3[] vertex, 
			Vector3 point,
			Vector3 planeNormal,
			Vector3 referencePoint)
		{
			double m1 = 0.0;
			double m2 = 0.0;
			double anglesum = 0.0;
			double costheta = 0.0;
		
			for(int i = 0; i < vertex.Length; i++){

				Vector3 projectP1 = vertex[i] - 
					(planeNormal * (Vector3.Dot(planeNormal, vertex[i]) +
						Vector3.Dot(planeNormal * -1.0, referencePoint)));

				Vector3 projectP2 = vertex[(i + 1) % vertex.Length] - 
					(planeNormal * (Vector3.Dot(planeNormal, vertex[(i + 1) % vertex.Length]) +
						Vector3.Dot(planeNormal * -1.0, referencePoint)));

				Vector3 p1 = projectP1  - point;
				Vector3 p2 = projectP2 - point;

				m1 = Vector3.Length(p1);
				m2 = Vector3.Length(p2);
				double prod = m1 * m2;

				//We are on a node, consider this inside 
				if(prod <= ConstValues.precision)
					return 2.0 * ConstValues.PI;
				else 
					costheta = Vector3.Dot( p1, p2 ) / prod;

				anglesum += Math.Acos(costheta);
			}
				
			return anglesum;
		}

		/// <summary>
		/// Tests if points are on the same plane.
		/// </summary>
		/// <returns><c>true</c>, if aligned plane points was tested, <c>false</c> otherwise.</returns>
		/// <param name="vt">Vt.</param>
		public static bool TestAlignedPlanePoints(
			Vector3[] vt)
		{
			if (vt == null || vt.Length != 4)
				return false;

			double[] t = new double[24];

			t [0] = vt [2].x * vt [1].y * vt [0].z;
			t [1] = vt [3].x * vt [1].y * vt [0].z;
			t [2] = vt [1].x * vt [2].y * vt [0].z;
			t [3] = vt [3].x * vt [2].y * vt [0].z;
			t [4] = vt [1].x * vt [3].y * vt [0].z;
			t [5] = vt [2].x * vt [3].y * vt [0].z;

			t [6] = vt [2].x * vt [0].y * vt [1].z;
			t [7] = vt [3].x * vt [0].y * vt [1].z;
			t [8] = vt [0].x * vt [2].y * vt [1].z;
			t [9] = vt [3].x * vt [2].y * vt [1].z;
			t [10] = vt [0].x * vt [3].y * vt [1].z;
			t [11] = vt [2].x * vt [3].y * vt [1].z;

			t [12] = vt [1].x * vt [0].y * vt [2].z;
			t [13] = vt [3].x * vt [0].y * vt [2].z;
			t [14] = vt [0].x * vt [1].y * vt [2].z;
			t [15] = vt [3].x * vt [1].y * vt [2].z;
			t [16] = vt [0].x * vt [3].y * vt [2].z;
			t [17] = vt [1].x * vt [3].y * vt [2].z;

			t [18] = vt [1].x * vt [0].y * vt [3].z;
			t [19] = vt [2].x * vt [0].y * vt [3].z;
			t [20] = vt [0].x * vt [1].y * vt [3].z;
			t [21] = vt [2].x * vt [1].y * vt [3].z;
			t [22] = vt [0].x * vt [2].y * vt [3].z;
			t [23] = vt [1].x * vt [2].y * vt [3].z;

			double d0 = -t [0] + t [1] + t [2] - t [3] - t [4] + t [5] +
			           t [6] - t [7] - t [8] + t [9] + t [10] - t [11] -
			           t [12] + t [13] + t [14] - t [15] - t [16] + t [17] +
			           t [18] - t [19] - t [20] + t [21] + t [22] - t [23];

			if (Math.Abs (d0) < ConstValues.precision)
				return true;

			return false;

		}

		/// <summary>
		/// Sign the specified a and b.
		/// </summary>
		/// <param name="a">The alpha component.</param>
		/// <param name="b">The blue component.</param>
		public static bool Sign(
			double a,
			double b)
		{
			if (a == 0.0 || b == 0.0)
				return true;

 			if (Math.Sign (a) == Math.Sign (b))
				return true;

			return false;
		}

		public static Vector3 GetPerpendicularVector(Vector3 a)
		{
			if (a.x == 0 && a.y == 0 && a.z == 0.0) 
			{
				throw new Exception ("Zero Vector");
			}

			if (a.x == 0)
				return new Vector3 (1.0, 0.0, 0.0);
			if (a.y == 0)
				return new Vector3 (0.0, 1.0, 0.0);
			if (a.z == 0)
				return new Vector3 (0.0, 0.0, 1.0);

			return new Vector3 (1.0, 1.0, -1.0 * (a.x + a.y) / a.z);
		}

		#endregion

	}
}

