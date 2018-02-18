using SharpEngineMathUtility;
	
namespace SharpPhysicsEngine.CollisionEngine
{
	internal struct SupportTriangle
	{
		#region Public Properties

		public Support A { get; private set; }
		public Support B { get; private set; }
		public Support C { get; private set; }
		public double S { get; private set; }
		public double T { get; private set; }
		public Vector3 Normal { get; private set; } 

		#endregion

		#region Constructor

		public SupportTriangle (
			Support a,
			Support b,
			Support c,
			double s,
			double t,
			Vector3 normal)
			:this()
		{
			this.A = a;
			this.B = b;
			this.C = c;
			this.S = s;
			this.T = t;
			this.Normal = normal;
		}

		#endregion

		#region Public Methods

		public void SetValueS (double s)
		{
			this.S = s;
		}

		public void SetValueT (double t)
		{
			this.T = t;
		}

		#endregion

	}
}

