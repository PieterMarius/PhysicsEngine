using System;

namespace PhysicsEngineMathUtility
{
	public struct Matrix3x3
	{

		#region Public Properties

		public readonly double r1c1;
		public readonly double r1c2;
		public readonly double r1c3;
		public readonly double r2c1;
		public readonly double r2c2;
		public readonly double r2c3;
		public readonly double r3c1;
		public readonly double r3c2;
		public readonly double r3c3;

		#endregion

		#region Constructors

		public Matrix3x3 (
			double r1c1, double r1c2, double r1c3,
			double r2c1, double r2c2, double r2c3,
			double r3c1, double r3c2, double r3c3)
		{
			this.r1c1 = r1c1; this.r1c2 = r1c2; this.r1c3 = r1c3;
			this.r2c1 = r2c1; this.r2c2 = r2c2; this.r2c3 = r2c3;
			this.r3c1 = r3c1; this.r3c2 = r3c2; this.r3c3 = r3c3; 
		}

		public Matrix3x3( double[] mat)
		{
			if (mat.Length == 8) 
			{
				this.r1c1 = mat [0]; this.r1c2 = mat [1]; this.r1c3 = mat [2];
				this.r2c1 = mat [3]; this.r2c2 = mat [4]; this.r2c3 = mat [5];
				this.r3c1 = mat [6]; this.r3c2 = mat [7]; this.r3c3 = mat [8]; 
			} 
			else 
			{
				throw new ArgumentException(COMPONENT_EXCEPTION);
			}
		}

		#endregion

		#region Public Methods

		public static Matrix3x3	ToZero()
		{
			return new Matrix3x3 (
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0);
		}

		public static Matrix3x3 IdentityMatrix()
		{
			return new Matrix3x3 (
				1.0, 0.0, 0.0,
				0.0, 1.0, 0.0,
				0.0, 0.0, 1.0);
		}

		public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b)
		{
			double r1x = (a.r1c1 * b.r1c1) + (a.r1c2 * b.r2c1) + (a.r1c3 * b.r3c1);
			double r1y = (a.r1c1 * b.r1c2) + (a.r1c2 * b.r2c2) + (a.r1c3 * b.r3c2);
			double r1z = (a.r1c1 * b.r1c3) + (a.r1c2 * b.r2c3) + (a.r1c3 * b.r3c3);

			double r2x = (a.r2c1 * b.r1c1) + (a.r2c2 * b.r2c1) + (a.r2c3 * b.r3c1);
			double r2y = (a.r2c1 * b.r1c2) + (a.r2c2 * b.r2c2) + (a.r2c3 * b.r3c2);
			double r2z = (a.r2c1 * b.r1c3) + (a.r2c2 * b.r2c3) + (a.r2c3 * b.r3c3);

			double r3x = (a.r3c1 * b.r1c1) + (a.r3c2 * b.r2c1) + (a.r3c3 * b.r3c1);
			double r3y = (a.r3c1 * b.r1c2) + (a.r3c2 * b.r2c2) + (a.r3c3 * b.r3c2);
			double r3z = (a.r3c1 * b.r1c3) + (a.r3c2 * b.r2c3) + (a.r3c3 * b.r3c3);

			return new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);
		}

		public static Matrix3x3 operator *(Matrix3x3 a, double d)
		{
			double r1x = a.r1c1 * d;
			double r1y = a.r1c2 * d;
			double r1z = a.r1c3 * d;

			double r2x = a.r2c1 * d;
			double r2y = a.r2c2 * d;
			double r2z = a.r2c3 * d;

			double r3x = a.r3c1 * d;
			double r3y = a.r3c2 * d;
			double r3z = a.r3c3 * d;

			return new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);
		}

		public static Vector3 operator*(Matrix3x3 a, Vector3 v)
		{
			Vector3 result = new Vector3 (
				                 a.r1c1 * v.x + a.r1c2 * v.y + a.r1c3 * v.z,
				                 a.r2c1 * v.x + a.r2c2 * v.y + a.r2c3 * v.z,
				                 a.r3c1 * v.x + a.r3c2 * v.y + a.r3c3 * v.z);
			return result;
		}

		public static Matrix3x3 operator+(Matrix3x3 a, Matrix3x3 b)
		{
			double r1x = a.r1c1 + b.r1c1;
			double r1y = a.r1c2 + b.r1c2;
			double r1z = a.r1c3 + b.r1c3;
				
			double r2x = a.r2c1 + b.r2c1;
			double r2y = a.r2c2 + b.r2c2;
			double r2z = a.r2c3 + b.r2c3;

			double r3x = a.r3c1 + b.r3c1;
			double r3y = a.r3c2 + b.r3c2;
			double r3z = a.r3c3 + b.r3c3;

			return new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);
		}

		/// <summary>
		/// Transpose the specified matrix.
		/// </summary>
		/// <param name="a">The alpha component.</param>
		public static Matrix3x3 Transpose(Matrix3x3 a)
		{

			double r1x = a.r1c1;
			double r1y = a.r2c1;
			double r1z = a.r3c1;

			double r2x = a.r1c2;
			double r2y = a.r2c2;
			double r2z = a.r3c2;

			double r3x = a.r1c3;
			double r3y = a.r2c3;
			double r3z = a.r3c3;

			return new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);
		}

		public Matrix3x3 Transpose()
		{
			return Matrix3x3.Transpose (this);
		}

		/// <summary>
		/// Invert the specified matrix.
		/// </summary>
		/// <param name="a">The alpha component.</param>
		public static Matrix3x3 Invert(Matrix3x3 a)
		{
			double den = 
				-(a.r1c3 * a.r2c2 * a.r3c1)
				+(a.r1c2 * a.r2c3 * a.r3c1)
				+(a.r1c3 * a.r2c1 * a.r3c2)
				-(a.r1c1 * a.r2c3 * a.r3c2)
				-(a.r1c2 * a.r2c1 * a.r3c3)
				+(a.r1c1 * a.r2c2 * a.r3c3);
			
			// Evaluate using of small value
			if (den != 0.0) {
				den = 1.0 / den;

				double r1c1 = (-(a.r2c3 * a.r3c2) + (a.r2c2 * a.r3c3)) * den;
				double r1c2 = ((a.r1c3 * a.r3c2) - (a.r1c2 * a.r3c3)) * den;
				double r1c3 = (-(a.r1c3 * a.r2c2) + (a.r1c2 * a.r2c3)) * den;

				double r2c1 = ((a.r2c3 * a.r3c1) - (a.r2c1 * a.r3c3)) * den;
				double r2c2 = (-(a.r1c3 * a.r3c1) + (a.r1c1 * a.r3c3)) * den;
				double r2c3 = ((a.r1c3 * a.r2c1) - (a.r1c1 * a.r2c3)) * den;

				double r3c1 = (-(a.r2c2 * a.r3c1) + (a.r2c1 * a.r3c2)) * den;
				double r3c2 = ((a.r1c2 * a.r3c1) - (a.r1c1 * a.r3c2)) * den;
				double r3c3 = (-(a.r1c2 * a.r2c1) + (a.r1c1 * a.r2c2)) * den;

				return new Matrix3x3 (
					r1c1, r1c2, r1c3,
					r2c1, r2c2, r2c3,
					r3c1, r3c2, r3c3);
			} 

			return a;
		}

		/// <summary>
		/// Normalizes the matrix rows.
		/// </summary>
		/// <returns>The rows.</returns>
		/// <param name="a">The alpha component.</param>
		public static Matrix3x3 NormalizeRows(Matrix3x3 a)
		{
			Vector3 r1 = new Vector3 (a.r1c1, a.r1c2, a.r1c3);
			Vector3 r2 = new Vector3 (a.r2c1, a.r2c2, a.r2c3);
			Vector3 r3 = new Vector3 (a.r3c1, a.r3c2, a.r3c3);
			r1 = Vector3.Normalize (r1);
			r2 = Vector3.Normalize (r2);
			r3 = Vector3.Normalize (r3);

			return new Matrix3x3 (
				r1.x, r1.y, r1.z,
				r2.x, r2.y, r2.z,
				r3.x, r3.y, r3.z);
		}

		/// <summary>
		/// Gets the rotation matrix.
		/// </summary>
		/// <returns>The rotation matrix.</returns>
		/// <param name="versor">Versor.</param>
		/// <param name="angle" in radian>Angle.</param>
		public static Matrix3x3 GetRotationMatrix(Vector3 versor, double angle)
		{
			Vector3 p = versor * versor;
			double c = Math.Cos(angle);
			double s = Math.Sin(angle);
			double t = 1.0 - c;

			double r1x = c + p.x * t;
			double r1y = (versor.x * versor.y * t) - (versor.z * s);
			double r1z = (versor.x * versor.z * t) + (versor.y * s);

			double r2x = (versor.y * versor.x * t) + (versor.z * s);
			double r2y = c + p.y * t;
			double r2z = (versor.y * versor.z * t) - (versor.x * s);

			double r3x = (versor.z * versor.x * t) - (versor.y * s);
			double r3y = (versor.z * versor.y * t) + (versor.x * s);
			double r3z = c + p.z * t;

			return new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);
		}

		public static Matrix3x3 GetSkewSymmetricMatrix(Vector3 input)
		{
			return new Matrix3x3 (
				0.0, -input.z, input.y,
				input.z, 0.0, -input.x,
				-input.y, input.x, 0.0);
		}

		public static Matrix3x3 GetRotationMatrix(Vector3 a, Vector3 b)
		{
			a = a.Normalize ();
			b = b.Normalize ();

			if (a != b && 
				a.Length () > 0 &&
				b.Length () > 0) 
			{
				Vector3 c = a.Cross (b);
				return new Matrix3x3 (
					a.x, b.x, c.x,
					a.y, b.y, c.y,
					a.z, b.z, c.z);

			} else {
				return new Matrix3x3 ();
			}
		}

		#endregion

		#region Const

		private const string COMPONENT_EXCEPTION = "Matrix must contain nine components";

		//TODO aggiungere eventuali altre eccezioni

		#endregion

	}
}

