using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
	internal class EngineCollisionPoint
	{
		#region Public Properties

		public Vector3 Dist { get; private set; }
		public VertexProperties A { get; private set; }
		public VertexProperties B { get; private set; }
		public Vector3 Normal { get; private set; }
		
		#endregion

		#region Constructor

		public EngineCollisionPoint (
			Vector3 dist,
			VertexProperties a,
			VertexProperties b,
			Vector3 normal)
		{
			this.Dist = dist;
			this.A = a;
			this.B = b;
			this.Normal = normal;
		}

		public EngineCollisionPoint()
		{}

		#endregion

		#region Public Methods

		public void SetA(VertexProperties a)
		{
			this.A = a;
		}

		public void SetB(VertexProperties b)
		{
			this.B = b;
		}

		public void SetDist(Vector3 dist)
		{
			this.Dist = dist;
		}
			
		public void SetNormal(Vector3 normal)
		{
			this.Normal = normal;
		}

		#endregion
	}
}

