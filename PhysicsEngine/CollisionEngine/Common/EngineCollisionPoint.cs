using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	public class EngineCollisionPoint
	{
		#region Public Properties

		public Vector3 dist { get; private set; }
		public VertexProperties a { get; private set; }
		public VertexProperties b { get; private set; }
		public Vector3 normal { get; private set; }
		
		#endregion

		#region Constructor

		public EngineCollisionPoint (
			Vector3 dist,
			VertexProperties a,
			VertexProperties b,
			Vector3 normal)
		{
			this.dist = dist;
			this.a = a;
			this.b = b;
			this.normal = normal;
		}

		public EngineCollisionPoint()
		{}

		#endregion

		#region Public Methods

		public void SetA(VertexProperties a)
		{
			this.a = a;
		}

		public void SetB(VertexProperties b)
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

