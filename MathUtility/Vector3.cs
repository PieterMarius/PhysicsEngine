using System;
using System.Collections.Generic;

namespace PhysicsEngineMathUtility
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
				this.x = vec [0];
				this.y = vec [1];
				this.z = vec [2];
			} 
			else 
			{
				throw new ArgumentException(COMPONENT_EXCEPTION);
			}
		}

		public Vector3(Vector3 v)
		{
			this.x = v.x;
			this.y = v.y;
			this.z = v.z;
		}


		#endregion

		#region Public Methods

		public static Vector3 operator+(Vector3 a, Vector3 b)
		{
			return new Vector3 (a.x + b.x, a.y + b.y, a.z + b.z);
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
				return new[] { this.x, this.y, this.z };
			}
		}

		public double this[ int index ]
		{
			get	
			{
				switch (index)
				{
				case 0:
					return this.x;
				case 1:
					return this.y;
				case 2:
					return this.z;
				default:
					throw new ArgumentException(COMPONENT_EXCEPTION, "index");
				}
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
				return false;
			
			return true;
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

		public static double Length(Vector3 a)
		{
			return Math.Sqrt (Dot (a, a));
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

		public static Vector3 RotatePoint(Vector3 a, Vector3 versor, double angle)
		{
			Vector3 p= new Vector3();
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


		#region IComparable implementation

		public int CompareTo (object obj)
		{
			if (obj is Vector3)
			{
				return this.CompareTo((Vector3)obj);
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
			return this == (Vector3)other;
		}

		public override bool Equals(object obj)
		{
			if(obj is Vector3)
			{
				Vector3 otherVector = (Vector3)obj;

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
				var hashCode = this.x.GetHashCode();
				hashCode = (hashCode * 397) ^ this.y.GetHashCode();
				hashCode = (hashCode * 397) ^ this.z.GetHashCode();
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

