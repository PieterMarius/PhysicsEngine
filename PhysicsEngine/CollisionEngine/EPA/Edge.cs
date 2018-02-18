
namespace SharpPhysicsEngine.CollisionEngine
{
	internal class Edge
	{
		public Support A { get; set; }
		public Support B { get; set; }

		public Edge (
			Support a,
			Support b)
		{
			this.A = a;
			this.B = b;
		}
	}
}

