using System;
using System.Collections.Generic;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal class Simplex : ICloneable, IEquatable<Simplex>
	{
		public List<Support> Support { get; set; }
		public double W { get; set; }
		public double T { get; set; }

		#region "Constructor"

		public Simplex ()
		{
			Support = new List<Support>();
		}

		public Simplex (
			List<Support> support,
			double w,
			double t)
		{
			Support = support;
			this.W = w;
			this.T = t;
		}

		#endregion

		#region "Public Methods"

		public bool AddSupport(Support sp)
		{
			if (Support.Exists(x => x.a == sp.a && x.b == sp.b))
				return false;

			Support.Add(sp);

			return true;
		}

		public object Clone()
		{
			//Support[] sup = new Support[4];
			//sup [0] = new Support (
			//	Support [0].s,
			//	Support [0].a,
			//	Support [0].b);

			//sup [1] = new Support (
			//	Support [1].s,
			//	Support [1].a,
			//	Support [1].b);

			//sup [2] = new Support (
			//	Support [2].s,
			//	Support [2].a,
			//	Support [2].b);

			//sup [3] = new Support (
			//	Support [3].s,
			//	Support [3].a,
			//	Support [3].b);

			//return new Simplex(sup, w, t);
			return new Simplex();
		}

		#endregion

		#region IEquatable implementation

		public bool Equals (Simplex other)
		{
			return (Support [0].a == other.Support [0].a &&
					Support [1].a == other.Support [1].a &&
					Support [2].a == other.Support [2].a &&
					Support [3].a == other.Support [3].a &&
					Support [0].b == other.Support [0].b &&
					Support [1].b == other.Support [1].b &&
					Support [2].b == other.Support [2].b &&
					Support [3].b == other.Support [3].b);
		}

		#endregion
	}
}

