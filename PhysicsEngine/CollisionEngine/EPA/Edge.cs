
namespace CollisionEngine
{
	public class Edge
	{
		public Support a { get; set; }
		public Support b { get; set; }

		public Edge (
			Support a,
			Support b)
		{
			this.a = a;
			this.b = b;
		}
	}
}

