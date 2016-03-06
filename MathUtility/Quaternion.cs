using System;

namespace PhysicsEngineMathUtility
{
	public struct Quaternion
	{
		#region Real Value

		public readonly double a;

		#endregion

		#region Complex Values

		public readonly double b;
		public readonly double c;
		public readonly double d;

		#endregion

		#region Constructors

		public Quaternion(
			double a, 
			double b, 
			double c, 
			double d)
		{
			this.a = a;
			this.b = b;
			this.c = c;
			this.d = d;
		}

		public Quaternion(double[] vec)
		{
			if (vec.Length == 4) 
			{
				this.a = vec [0];
				this.b = vec [1];
				this.c = vec [2];
				this.d = vec [3];
			} 
			else 
			{
				throw new ArgumentException(COMPONENT_EXCEPTION);
			}
		}

		/// <summary>
		/// Initializes a new instance of the <see cref="MathUtility.Quaternion"/> class.
		/// </summary>
		/// <param name="versor">Versor.</param>
		/// <param name="angle in radian">Angle.</param>
		public Quaternion(Vector3 versor, double angle)
		{
			double coeff = Math.Sin (angle * 0.5);

			this.a = Math.Cos (angle * 0.5);
			this.b = versor.x * coeff;
			this.c = versor.y * coeff;
			this.d = versor.z * coeff;
		}

		#endregion

		#region Public Methods

		public static Quaternion operator*(Quaternion q1, Quaternion q2)
		{
			double ra = (q2.a * q1.a - q2.b * q1.b - q2.c * q1.c - q2.d * q1.d);
			double rb = (q2.a * q1.b + q2.b * q1.a - q2.c * q1.d + q2.d * q1.c);
			double rc = (q2.a * q1.c + q2.b * q1.d + q2.c * q1.a - q2.d * q1.b);
			double rd = (q2.a * q1.d - q2.b * q1.c + q2.c * q1.b + q2.d * q1.a);

			return new Quaternion (ra, rb, rc, rd);
		}

		public static Quaternion operator+(Quaternion q1, Quaternion q2)
		{
			double ra = q1.a + q2.a;
			double rb = q1.b + q2.b;
			double rc = q1.c + q2.c;
			double rd = q1.d + q2.d;

			return new Quaternion (ra, rb, rc, rd);
		}

		public static Quaternion operator-(Quaternion q1, Quaternion q2)
		{
			double ra = q1.a - q2.a;
			double rb = q1.b - q2.b;
			double rc = q1.c - q2.c;
			double rd = q1.d - q2.d;

			return new Quaternion (ra, rb, rc, rd);
		}


		/// <summary>
		/// Gets the vector.
		/// </summary>
		/// <returns>The vector.</returns>
		/// <param name="q">Q.</param>
		public static Vector3 GetVector(Quaternion q1, Quaternion q2)
		{
			double x = (q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c);
			double y = (q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b);
			double z = (q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a);

			return new Vector3 (x, y, z);
		}

		public static Quaternion GetQuaternion(Matrix3x3 a)
		{
			double tr = a.r1c1 + a.r2c2 + a.r3c3;

			if (tr >= 0.0) {
				double s = Math.Sqrt(tr + 1.0);
				double ra = 0.5 * s;
				s = 0.5 / s;
				double rb = (a.r3c2 - a.r2c3) * s;
				double rc = (a.r1c3 - a.r3c1) * s;
				double rd = (a.r2c1 - a.r1c2) * s;

				return new Quaternion (ra, rb, rc, rd);
			} 
				
			if (a.r2c2 > a.r1c1) {
				double s = Math.Sqrt ((a.r2c2 - (a.r3c3 + a.r1c1)) + 1.0);
				double rc = 0.5 * s;
				s = 0.5 / s;
				double rd = (a.r2c3 + a.r3c2) * s;
				double rb = (a.r1c2 + a.r2c1) * s;
				double ra = (a.r1c3 - a.r3c1) * s;

				return new Quaternion (ra, rb, rc, rd);
			}
				
			if ((a.r3c3 > a.r1c1) || (a.r3c3 > a.r2c2)) {
				double s = Math.Sqrt ((a.r3c3 - (a.r1c1 + a.r2c2)) + 0.5);
				double rd = 0.5 * s;
				s = 0.5 / s;
				double rb = (a.r3c1 + a.r1c3) * s;
				double rc = (a.r2c3 + a.r3c2) * s;
				double ra = (a.r2c1 - a.r1c2) * s;

				return new Quaternion (ra, rb, rc, rd);
			} 

			else 
			{
				double s = Math.Sqrt ((a.r1c1 - (a.r2c2 + a.r3c3)) + 1.0);
				double rb = 0.5 * s;
				s = 0.5 / s;
				double rc = (a.r1c2 + a.r2c1) * s;
				double rd = (a.r3c1 + a.r1c3) * s;
				double ra = (a.r3c2 - a.r2c3) * s;

				return new Quaternion (ra, rb, rc, rd);
			}
		}

		public static Quaternion Inverse(Quaternion q) 
		{
			double d = 1.0 / (q.a * q.a + q.b * q.b + q.c * q.c + q.d * q.d);
			return new Quaternion (q.a * d, -q.b * d, -q.c * d, -q.d * d);
		}

		public Quaternion Inverse()
		{
			double d = 1.0 / (this.a * this.a + this.b * this.b + this.c * this.c + this.d * this.d);
			return new Quaternion (this.a * d, -this.b * d, -this.c * d, -this.d * d);
		}

		public static Matrix3x3 ConvertToMatrix(Quaternion q)
		{
			double xx = 2.0 * q.b * q.b;
			double yy = 2.0 * q.c * q.c;
			double zz = 2.0 * q.d * q.d;
			double s = q.a;

			double r1x = 1.0 - yy - zz;
			double r1y = 2.0 * (q.b * q.c - s * q.d);
			double r1z = 2.0 * (q.b * q.d + s * q.c);

			double r2x = 2.0 * (q.b * q.c + s * q.d);
			double r2y = 1.0 - xx - zz;
			double r2z = 2.0 * (q.c * q.d - s * q.b);

			double r3x = 2.0 * (q.b * q.d - s * q.c);
			double r3y = 2.0 * (q.c * q.d + s * q.b);
			double r3z = 1.0 - xx - yy;

			return new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);

		}

		public static double Length(Quaternion q)
		{
			return Math.Sqrt (
				q.a * q.a +
				q.b * q.b +
				q.c * q.c +
				q.d * q.d);
		}

		public static Quaternion Normalize(Quaternion q)
		{
			double l = Quaternion.Length (q);

			if (l > 0.0) {
				
				double inv = 1.0 / l;
				double ra = q.a * inv;
				double rb = q.b * inv;
				double rc = q.c * inv;
				double rd = q.d * inv;

				return new Quaternion (ra, rb, rc, rd);
			}

			return q;
		}

		public static Quaternion IntegrateQuaternion(
			Quaternion q, 
			Vector3 v, 
			double delta)
		{
			Vector3 theta = v * (delta * 0.5);

			double ra = 0.0;
			double rb = theta.x;
			double rc = theta.y;
			double rd = theta.z;

			return Quaternion.Normalize (new Quaternion(ra,rb,rc,rd) * q);
		}

		public static Vector3 GetEuler(Quaternion q)
		{
			
			double test = q.b * q.c + q.d * q.a;
			double heading, attitude, bank;

			// singularity at north pole
			if (test > 0.499999) 
			{ 
				heading = 2.0 * Math.Atan2 (q.b, q.a);
				attitude = Math.PI / 2.0;
				bank = 0.0;
				return new Vector3 (bank, heading, attitude);
			}

			// singularity at south pole
			if (test < -0.499999) { 
				heading = -2.0 * Math.Atan2 (q.b, q.a);
				attitude = -Math.PI / 2.0;
				bank = 0.0;
				return new Vector3 (bank, heading, attitude);
			}

			double sqx = q.b * q.b;
			double sqy = q.c * q.c;
			double sqz = q.d * q.d;

			// y-axis
			heading = Math.Atan2 (2.0 * q.c * q.a - 2.0 * q.b * q.d, 
				1.0 - 2.0 * sqy - 2.0 * sqz);

			// z-axis
			attitude = Math.Asin (2.0 * test); 

			// x-axis
			bank = Math.Atan2 (2.0 * q.b * q.a - 2.0 * q.c * q.d, 
				1.0 - 2.0 * sqx - 2.0 * sqz); 
			
			return new Vector3 (bank, heading, attitude);
		}

		#endregion

		#region Const

		private const string COMPONENT_EXCEPTION = "Quaternion must contain four components (a,b,c,d) ";

		//TODO aggiungere eventuali altre eccezioni

		#endregion
			
	}
}

