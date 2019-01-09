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

using System;
using System.Linq;

namespace SharpEngineMathUtility
{
	public static class GeometryUtils
	{
		
		#region Public Static Methods
		
		/// <summary>
		/// Gets the projected point on line.
		/// </summary>
		/// <returns>The projected point on line.</returns>
		/// <param name="a">The alpha component.</param>
		/// <param name="b">The blue component.</param>
		/// <param name="p">P.</param>
		/// <param name="t">T.</param>
		public static Vector3d GetProjectedPointOnLine(
			Vector3d a, 
			Vector3d b, 
			Vector3d p,
			ref double  t) 
		{
			Vector3d c = b - a;
			Vector3d d = p - a;
			double mod = Vector3d.Length(c);
            t = c.Dot(d) / (mod * mod);

			if (t < 0.0) 
				return a;
			if (t > 1.0) 
				return b;

			return (a + ( c * t ));
		}

		/// <summary>
		/// Tests the edges intersect.
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
		public static bool TestEdgesIntersect(
			Vector3d p1, 
			Vector3d p2, 
			Vector3d p3, 
			Vector3d p4, 
			ref Vector3d pa, 
			ref Vector3d pb, 
			ref double mua, 
			ref double mub) 
		{
			Vector3d p13, p43, p21;
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
			
			d1343 = p13.Dot(p43);
			d4321 = p43.Dot(p21);
			d1321 = p13.Dot(p21);
			d4343 = p43.Dot(p43);
            d2121 = p21.Dot(p21);

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
		public static Vector3d CalculateTriangleNormal(
			Vector3d a,
			Vector3d b,
			Vector3d c)
		{
			Vector3d v1 = b - a;
			Vector3d v2 = c - a;

			double x = (v1.y * v2.z) - (v1.z * v2.y);
			double y = -((v2.z * v1.x) - (v2.x * v1.z));
			double z = (v1.x * v2.y) - (v1.y * v2.x);

			return Vector3d.Normalize(new Vector3d(x, y, z));
		}
			
		/// <summary>
		/// Projects the vector on plane.
		/// </summary>
		/// <returns>The vector on plane.</returns>
		/// <param name="normal">Normal.</param>
		public static Vector3d ProjectVectorOnPlane(Vector3d normal)
		{
			if (Math.Abs (normal.x) < 0.000000001) 
				return new Vector3d (1.0, 0.0, 0.0);
				
			if (Math.Abs (normal.y) < 0.000000001) 
				return new Vector3d (0.0, 1.0, 0.0);
				
			if (Math.Abs (normal.z) < 0.000000001) 
				return new Vector3d (0.0, 0.0, 1.0);

			double z = -(normal.x + normal.y) / normal.z;

			return Vector3d.Normalize(new Vector3d (1.0, 1.0, z));
		}

		public static void ComputeBasis(
			Vector3d normal,
			ref Vector3d a,
			ref Vector3d b)
		{
			if (Math.Abs(normal.x) >= 0.577350269189)
				a = new Vector3d(normal.y, -normal.x, 0.0);
			else
				a = new Vector3d(0.0, normal.z, -normal.y);

			a = a.Normalize();
			b = normal.Cross(a).Normalize();
		}
                        		
		public static Vector3d GetRandomDirection()
		{
			double angle = MathUtils.GetRandom(0.0, ConstValues.PI2);
			double z = MathUtils.GetRandom(-1.0, 1.0);
			double sqrt = Math.Sqrt(1.0 - z * z);

			return new Vector3d(
				sqrt * Math.Cos(angle),
				sqrt * Math.Sin(angle),
				z);
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
		public static Vector3d? GetPointTriangleIntersection(
			Vector3d v1,
			Vector3d v2,
			Vector3d v3,
			Vector3d p,
			ref double t0,
			ref double t1)
		{
			Vector3d diff = p - v1;
			Vector3d edge0 = v2 - v1;
			Vector3d edge1 = v3 - v1;
			double a00 = edge0.Dot(edge0);
			double a01 = edge0.Dot(edge1);
			double a11 = edge1.Dot(edge1);
			double b0 = -diff.Dot(edge0);
			double b1 = -diff.Dot(edge1);
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
					if (Math.Abs(det) > 1E-100) {
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

			return v1 + t0 * edge0 + t1 * edge1;
		}

		/// <summary>
		/// Tests collinearity.
		/// </summary>
		/// <returns><c>true</c>, if collinearity was tested, <c>false</c> otherwise.</returns>
		/// <param name="a">The alpha component.</param>
		/// <param name="b">The blue component.</param>
		/// <param name="c">C.</param>
		public static bool TestCollinearity(
			Vector3d a,
			Vector3d b,
			Vector3d c)
		{
			Vector3d d = b - a;
			Vector3d e = c - a;

			Vector3d f =  d.Cross(e);
			if (Math.Abs (f.x) < ConstValues.collinearityTolerance &&
				Math.Abs (f.y) < ConstValues.collinearityTolerance &&
				Math.Abs (f.z) < ConstValues.collinearityTolerance)
				return true;

			return false;
		}

		/// <summary>
		/// Gets barycentric point.
		/// </summary>
		/// <returns>The barycentric point.</returns>
		/// <param name="vertex">Vertex.</param>
		public static Vector3d GetBarycentricPoint(Vector3d[] vertex)
		{
			var t = new Vector3d ();
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
			Vector3d a,
			Vector3d b,
			Vector3d normal)
		{
			Vector3d c = a.Cross(b);

			double angle = Math.Atan2 (c.Length(), a.Dot(b));

			return c.Dot(normal) < 0.0 ? -angle : angle; 

		}

		/// <summary>
		/// Turns vector clock wise.
		/// </summary>
		/// <returns>The vector clock wise.</returns>
		/// <param name="vertices">Vertices.</param>
		/// <param name="normal">Normal.</param>
		public static Vector3d[] TurnVectorClockWise(
			Vector3d[] vertices,
			Vector3d normal)
		{
			if (vertices.Length > 0) {
				Vector3d barycentricPoint = GetBarycentricPoint (vertices);

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

				Vector3d[] result = new Vector3d[vert.Length];

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
			Vector3d[] vertex, 
			Vector3d point,
			Vector3d planeNormal,
			Vector3d referencePoint)
		{
			double m1 = 0.0;
			double m2 = 0.0;
			double anglesum = 0.0;
			double costheta = 0.0;
		
			for(int i = 0; i < vertex.Length; i++){

				Vector3d projectP1 = vertex[i] - 
					(planeNormal * (Vector3d.Dot(planeNormal, vertex[i]) +
						Vector3d.Dot(planeNormal * -1.0, referencePoint)));

				Vector3d projectP2 = vertex[(i + 1) % vertex.Length] - 
					(planeNormal * (Vector3d.Dot(planeNormal, vertex[(i + 1) % vertex.Length]) +
						Vector3d.Dot(planeNormal * -1.0, referencePoint)));

				Vector3d p1 = projectP1  - point;
				Vector3d p2 = projectP2 - point;

				m1 = p1.Length();
				m2 = p2.Length();
				double prod = m1 * m2;

                //We are on a node, consider this inside 
                if (prod <= ConstValues.precision)
                    return 2.0 * ConstValues.PI;
                else
                    costheta = p1.Dot(p2) / prod;

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
			Vector3d[] vt)
		{
			if (vt == null || vt.Length < 4)
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
			if (a.Equals(0.0) || b.Equals(0.0))
				return true;

			if (Math.Sign (a) == Math.Sign (b))
				return true;

			return false;
		}

		public static Vector3d GetPerpendicularVector(Vector3d a)
		{
			if (a.x.Equals(0.0) && 
				a.y.Equals(0.0) && 
				a.z.Equals(0.0)) 
			{
				throw new Exception ("Zero Vector");
			}

			if (a.x.Equals(0))
				return new Vector3d (1.0, 0.0, 0.0);
			if (a.y.Equals(0))
				return new Vector3d (0.0, 1.0, 0.0);
			if (a.z.Equals(0))
				return new Vector3d (0.0, 0.0, 1.0);

			return new Vector3d (1.0, 1.0, -1.0 * (a.x + a.y) / a.z);
		}

		public static Vector3d TurnOutVNormal(
			Vector3d normal,
			Vector3d v)
		{
			Vector3d resNormal = normal;
			if (Vector3d.Dot(normal, v) < 0.0)
				resNormal = normal * -1.0;
						
			return resNormal;
		}

		public static Vector3d? RayTriangleIntersection(
			Vector3d v0,
			Vector3d v1,
			Vector3d v2,
			Vector3d point,
			Vector3d direction,
			bool bothDirection = false)
		{
			Vector3d e1 = v1 - v0;
			Vector3d e2 = v2 - v0;

			Vector3d h = direction.Cross(e2);

            double a = e1.Dot(h);

			if (a > -0.000001 &&
				a < 0.000001)
				return null;

			double f = 1.0 / a;

			Vector3d s = point - v0;

			double u = f * s.Dot(h);

			if (u < 0.0 || u > 1.0)
				return null;

			Vector3d q = s.Cross(e1);
            double v = f * direction.Dot(q);

			if (v < 0.0 || u + v > 1.0)
				return null;

			double t = f * e2.Dot(q);

			if (t > 0.000001 || 
				(bothDirection && t < -0.000001))
			    return point + t * direction;
			else
				return null;
		}

        public static double[][] GetArrayFromVector2(Vector2d[] array)
        {
            var result = new double[array.Length][];

            for (int i = 0; i < array.Length; i++)
                result[i] = array[i].Array;

            return result;
        }

        public static double[][][] GetMatrixFromVector2Matrix(Vector2d[][] matrix)
        {
            var result = new double[matrix.Length][][];

            for (int i = 0; i < matrix.Length; i++)
                result[i] = GetArrayFromVector2(matrix[i]);

            return result;
        }

        public static Vector3d[] GetVector3ArrayFromMatrix(double[][] input)
        {
            var result = new Vector3d[input.Length];

            for (int i = 0; i < input.Length; i++)
            {
                result[i] = new Vector3d(input[i]);
            }

            return result;
        }

        #endregion

    }
}

