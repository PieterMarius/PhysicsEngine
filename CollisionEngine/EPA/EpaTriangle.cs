using System;
using PhysicsEngineMathUtility;
	
namespace CollisionEngine
{
	public struct EpaTriangle
	{
		#region Public Properties

		public Support a { get; private set; }
		public Support b { get; private set; }
		public Support c { get; private set; }
		public double s { get; private set; }
		public double t { get; private set; }
		public Vector3 normal { get; private set; } 

		#endregion

		#region Constructor

		public EpaTriangle (
			Support a,
			Support b,
			Support c,
			double s,
			double t,
			Vector3 normal)
			:this()
		{
			this.a = a;
			this.b = b;
			this.c = c;
			this.s = s;
			this.t = t;
			this.normal = normal;
		}

		#endregion

		#region Public Methods

		public void SetValueS (double s)
		{
			this.s = s;
		}

		public void SetValueT (double t)
		{
			this.t = t;
		}

		#endregion


	}
}

