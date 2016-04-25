using System;
using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public class ObjectGeometry
	{
		#region Object Properties

		public Vector3[] VertexPosition { get; private set; }
		public Vector3[] VertexInitialPosition { get; private set; }
		public int[][] Triangle { get; private set; }
		public AABB AABBox { get; private set; }

		#endregion

		#region Constructor

		public ObjectGeometry (
			Vector3[] inputVertexPosition,
			Vector3[] inputVertexInitialPosition,
			int[][] inputTriangle)
		{
			this.VertexPosition = new Vector3[inputVertexPosition.Length];
			this.VertexInitialPosition = new Vector3[inputVertexInitialPosition.Length];

			for (int i = 0; i < VertexPosition.Length; i++) 
			{
				this.VertexPosition [i] = inputVertexPosition [i];
				this.VertexInitialPosition [i] = inputVertexInitialPosition [i];
			}

			this.Triangle = new int[inputTriangle.Length][];

			for (int i = 0; i < inputTriangle.Length; i++) 
			{
				this.Triangle [i] = new int[inputTriangle [i].Length];
				for (int j = 0; j < inputTriangle [i].Length; j++) 
				{
					this.Triangle [i] [j] = inputTriangle [i] [j];
				}
			}
		}

		#endregion

		#region Public Methods

		public void SetVertexPosition(Vector3 v, int index)
		{
			if (this.VertexPosition != null && 
				this.VertexPosition.Length > index ) 
			{
				this.VertexPosition [index] = v;
			}
		}
			
		public void SetVertexInitialPosition(Vector3 v, int index)
		{
			if (this.VertexInitialPosition != null && 
				this.VertexInitialPosition.Length > index) 
			{
				this.VertexInitialPosition [index] = new Vector3 (v.x, v.y, v.z);
			}
		}

		public void SetVertexPositions(Vector3[] v)
		{
			this.VertexPosition = new Vector3[v.Length];
			for (int i = 0; i < v.Length; i++)
				this.VertexPosition [i] = v[i];
		}

		public void SetVertexInitialPositions(Vector3[] v)
		{
			this.VertexInitialPosition = new Vector3[v.Length];
			for (int i = 0; i < v.Length; i++)
				this.VertexInitialPosition [i] = v [i];
		}

		public void SetAABB(AABB box)
		{
			this.AABBox = box;
		}
			
		#endregion

	}
}

