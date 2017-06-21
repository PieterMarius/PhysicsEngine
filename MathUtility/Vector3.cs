using System;
using System.Collections.Generic;

namespace SharpEngineMathUtility
{
	public struct Vector3: IComparable, IEquatable<Vector3>
	{
		#region Public Properties

		public readonly double x;
		public readonly double y;
		public readonly double z;

		#endregion

		#region Constructors

		public Vector3 (
			double x, 
			double y, 
			double z)
		{
			this.x = x;
			this.y = y;
			this.z = z;
		}

		public Vector3(double[] vec)
		{
			if (vec.Length == 3) 
			{
				x = vec [0];
				y = vec [1];
				z = vec [2];
			} 
			else 
			{
				throw new ArgumentException(COMPONENT_EXCEPTION);
			}
		}

		public Vector3(Vector3 v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
		}


		#endregion

		#region Public Methods

		public static Vector3 operator+(Vector3 a, Vector3 b)
		{
			return new Vector3 (a.x + b.x, a.y + b.y, a.z + b.z);
		}

		public static Vector3 operator+(Vector3 a, double b)
		{
			return new Vector3 (a.x + b, a.y + b, a.z + b);
		}

		public static Vector3 operator+(double b, Vector3 a)
		{
			return new Vector3 (a.x + b, a.y + b, a.z + b);
		}

		public static Vector3 operator-(Vector3 a, Vector3 b)
		{
			return new Vector3 (a.x - b.x, a.y - b.y, a.z - b.z);
		}

		public static Vector3 operator*(Vector3 a, Vector3 b)
		{
			return new Vector3 (a.x * b.x, a.y * b.y, a.z * b.z);
		}

		public static Vector3 operator*(Vector3 a, double b)
		{
			return new Vector3 (a.x * b, a.y * b, a.z * b);
		}

		public static Vector3 operator*(double b, Vector3 a)
		{
			return new Vector3 (a.x * b, a.y * b, a.z * b);
		}

		public static Vector3 operator/(Vector3 a, Vector3 b)
		{
			return new Vector3 (a.x / b.x, a.y / b.y, a.z / b.z);
		}

		public static Vector3 operator/(Vector3 a, double d)
		{
			return new Vector3 (a.x / d, a.y / d, a.z / d);
		}

		public static Vector3 operator/(double d, Vector3 a)
		{
			return new Vector3 (a.x / d, a.y / d, a.z / d);
		}

		public double[] Array
		{
			get
			{
				return new[] { x, y, z };
			}
		}

		public double this[ int index ]
		{
			get	
			{
				switch (index)
				{
				case 0:
					return x;
				case 1:
					return y;
				case 2:
					return z;
				default:
					throw new ArgumentException(COMPONENT_EXCEPTION, nameof(index));
				}
			}
		}

		public List<double> ToList
		{
			get
			{
				return new List<double>() { x, y, z };
			}
		}

		public static bool operator>(Vector3 a, Vector3 b)
		{
			if (Length (a) + ConstValues.precision > Length (b) + ConstValues.precision)
				return true;
			
			return false;
		}

		public static bool operator==(Vector3 a, Vector3 b)
		{
			if (Math.Abs (a.x - b.x) < ConstValues.precision &&
				Math.Abs (a.y - b.y) < ConstValues.precision &&
				Math.Abs (a.z - b.z) < ConstValues.precision)
				return true;
			
			return false;
		}

		public static bool operator!=(Vector3 a, Vector3 b)
		{
			if (!(a == b))
				return true;
			
			return false;
		}

		public static bool operator<(Vector3 a, Vector3 b)
		{
			if (Length (a) + ConstValues.precision < Length (b) + ConstValues.precision)
				return true;
			
			return false;
		}

		public static bool operator<=(Vector3 a, Vector3 b)
		{
			if (Length (a) + ConstValues.precision <= Length (b) + ConstValues.precision)
				return true;

			return false;
		}

		public static bool operator>=(Vector3 a, Vector3 b)
		{
			if (Length (a) + ConstValues.precision >= Length (b) + ConstValues.precision)
				return true;

			return false;
		}

		public static double Dot(Vector3 a, Vector3 b)
		{
			return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
		}

		public double Dot(Vector3 a)
		{
			return (x * a.x) + (y * a.y) + (z * a.z);
		}

		public static double Length(Vector3 a)
		{
			return Math.Sqrt (Dot (a, a));
		}

		public double Length()
		{
			return Math.Sqrt (Dot (this));
		}

		public static Vector3 ToZero()
		{
			return new Vector3 (0.0, 0.0, 0.0);
		}

		public static Vector3 Normalize(Vector3 a)
		{
			double l = Length (a);
			if (l > 0.0) {
				
				l = 1.0 / l;

				return new Vector3 (
					a.x * l, 
					a.y * l, 
					a.z * l);
			}

			return a;
		}

		public Vector3 Normalize()
		{
			return Normalize (this);
		}

		public static Vector3 Zero()
		{
			return new Vector3(0.0, 0.0, 0.0);
		}

		public static Vector3 RotatePoint(Vector3 a, Vector3 versor, double angle)
		{
			var p= new Vector3();
			p = versor * versor;
			double c = Math.Cos (angle);
			double s = Math.Sin (angle);
			double t = 1.0 - angle;

			double X = (a.x * (p.x + (1.0 - p.x) * c))+(a.y * ((t * versor.x * versor.y)-(s * versor.z)))+(a.z * ((t * versor.x * versor.z)+(s * versor.y)));
			double Y = (a.x * ((t * versor.x * versor.y)+(s * versor.z)))+(a.y * (p.y + (1.0 - p.y) * c))+(a.z * ((t * versor.y * versor.z)-(s * versor.x)));
			double Z = (a.x * ((t * versor.z * versor.x)-(s * versor.y)))+(a.y * ((t * versor.z * versor.y)+(s * versor.x)))+(a.z * (p.z + ( 1.0 - p.z) * c));

			return new Vector3 (X, Y, Z);
		}

		public static Vector3 Cross(Vector3 a, Vector3 b) {
			return new Vector3(
				(a.y * b.z)-(a.z * b.y),
				(a.z * b.x)-(a.x * b.z),
				(a.x * b.y)-(a.y * b.x));
		}

		public Vector3 Cross(Vector3 a)
		{
			return Cross (this, a);
		}

		public static Vector3 ProjectVectorOnPlane(Vector3 normal)
		{
			if (Math.Abs(normal.x) < ConstValues.precision)
				return new Vector3 (1.0, 0.0, 0.0);

			if (Math.Abs(normal.y) < ConstValues.precision) 
				return new Vector3 (0.0, 1.0, 0.0);

			if (Math.Abs(normal.z) < ConstValues.precision) 
				return new Vector3 (0.0, 0.0, 1.0);

			double X = 1.0;
			double Y = 1.0;
			double Z = -(normal.x * X + normal.y * Y) / normal.z;

			return Normalize (new Vector3 (X, Y, Z));
		}

		public static bool TestIfPointAreAligned(Vector3 a, Vector3 b, Vector3 c)
		{
			double det = 
				-(a.z * b.y * c.x) + 
				(a.y * b.z * c.x) + 
				(a.z * b.x * c.y) - 
				(a.x * b.z * c.y) - 
				(a.y * b.x * c.z) + 
				(a.x * b.y * c.z);

			if (Math.Abs(det) < ConstValues.precision) 
				return true;
			return false;
		}

		public Matrix3x3 GetSkewSymmetricMatrix()
		{
			return new Matrix3x3 (
				0.0, -z, y,
				z, 0.0, -x,
				-y, x, 0.0);
		}

		public static Vector3 UniformSign(Vector3 input, Vector3 sign)
		{
			if (input.Dot(sign) >= 0.0)
				return input;

			double a = input.x;
			double b = input.y;
			double c = input.z;

			if (Math.Sign(input.x) != Math.Sign(sign.x) && 
			   Math.Sign(sign.x) != 0)
				a = -input.x;
			
			if (Math.Sign(input.y) != Math.Sign(sign.y) &&
			   Math.Sign(sign.y) != 0)
				b = -input.y;

			if (Math.Sign(input.z) != Math.Sign(sign.z) &&
			   Math.Sign(sign.z) != 0)
				c = -input.z;

			return new Vector3(a, b, c);
		}

		#region IComparable implementation

		public int CompareTo (object obj)
		{
			if (obj is Vector3)
			{
				return CompareTo((Vector3)obj);
			}

			throw new ArgumentException ();
		}

		public int CompareTo(Vector3 other)
		{
			if (this < other)
			{
				return -1;
			}

			if (this > other)
			{
				return 1;
			}

			return 0;
		}

		#endregion


		#region IEquatable implementation

		public bool Equals (Vector3 other)
		{
			return this == other;
		}

		public override bool Equals(object obj)
		{
			if(obj is Vector3)
			{
				var otherVector = (Vector3)obj;

				return otherVector.Equals(this);
			}
			else
			{
				return false;
			}
		}

		public override int GetHashCode()
		{
			unchecked
			{
				var hashCode = x.GetHashCode();
				hashCode = (hashCode * 397) ^ y.GetHashCode();
				hashCode = (hashCode * 397) ^ z.GetHashCode();
				return hashCode;
			}
		}

		#endregion

		#region Const

		private const string COMPONENT_EXCEPTION = "Vector must contain three components (x,y,z)";

		//TODO aggiungere eventuali altre eccezioni

		#endregion

		#endregion
	}
}

