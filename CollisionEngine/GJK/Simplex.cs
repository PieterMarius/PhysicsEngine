using System;

namespace CollisionEngine
{
	public class Simplex : ICloneable, IEquatable<Simplex>
	{
		public Support[] Support { get; set; }
		public double w { get; set; }
		public double t { get; set; }

		#region "Constructor"

		public Simplex ()
		{
			this.Support = new Support[4];
		}

		public Simplex (
			Support[] support,
			double w,
			double t)
		{
			this.Support = support;
			this.w = w;
			this.t = t;
		}

		#endregion

		#region "Public Methods"

		public object Clone()
		{
			Support[] sup = new Support[4];
			sup [0] = new Support (
				this.Support [0].s,
				this.Support [0].a,
				this.Support [0].b);

			sup [1] = new Support (
				this.Support [1].s,
				this.Support [1].a,
				this.Support [1].b);

			sup [2] = new Support (
				this.Support [2].s,
				this.Support [2].a,
				this.Support [2].b);

			sup [3] = new Support (
				this.Support [3].s,
				this.Support [3].a,
				this.Support [3].b);
			
			return new Simplex (
				sup, 
				this.w, 
				this.t);
		}

		#endregion

		#region IEquatable implementation

		public bool Equals (Simplex other)
		{
			return (this.Support [0].a == other.Support [0].a &&
					this.Support [1].a == other.Support [1].a &&
					this.Support [2].a == other.Support [2].a &&
					this.Support [3].a == other.Support [3].a &&
					this.Support [0].b == other.Support [0].b &&
					this.Support [1].b == other.Support [1].b &&
					this.Support [2].b == other.Support [2].b &&
					this.Support [3].b == other.Support [3].b);
		}

		#endregion
	}
}

