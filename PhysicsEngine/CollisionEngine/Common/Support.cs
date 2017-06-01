using SharpEngineMathUtility;

namespace SharpPhysicsEngine.CollisionEngine
{
	public struct Support
	{
		public readonly Vector3 s;
		public readonly int a;
		public readonly int b;

		public Support (
			Vector3 s,
			int a,
			int b)
		{
			this.s = s;
			this.a = a;
			this.b = b;
		}
			
	}
}

