using PhysicsEngineMathUtility;

namespace CollisionEngine
{
	public class EpaCollisionPoint
	{
		#region Public Properties

		public Vector3 dist { get; private set; }
		public Vector3 a { get; private set; }
		public Vector3 b { get; private set; }
		public Vector3 normal { get; private set; }

		#endregion

		#region Constructor

		public EpaCollisionPoint (
			Vector3 dist,
			Vector3 a,
			Vector3 b,
			Vector3 normal)
		{
			this.dist = dist;
			this.a = a;
			this.b = b;
			this.normal = normal;
		}

		#endregion

		#region Public Methods

		public void SetA(Vector3 a)
		{
			this.a = a;
		}

		public void SetB(Vector3 b)
		{
			this.b = b;
		}

		public void SetDist(Vector3 dist)
		{
			this.dist = dist;
		}
			
		public void SetNormal(Vector3 normal)
		{
			this.normal = normal;
		}

		#endregion
	}
}

