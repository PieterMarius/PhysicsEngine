using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public class ObjectGeometry
	{
		#region Object Properties

		public Vector3[] VertexPosition { get; private set; }
		public int[][] Triangle { get; private set; }
		public AABB AABBox { get; private set; }

		#endregion

		#region Constructor

		public ObjectGeometry (
			Vector3[] inputVertexPosition,
			int[][] inputTriangle)
		{
			VertexPosition = new Vector3[inputVertexPosition.Length];

			for (int i = 0; i < VertexPosition.Length; i++) 
			{
				VertexPosition [i] = inputVertexPosition [i];
			}

			Triangle = new int[inputTriangle.Length][];

			for (int i = 0; i < inputTriangle.Length; i++) 
			{
				Triangle [i] = new int[inputTriangle [i].Length];
				for (int j = 0; j < inputTriangle [i].Length; j++) 
				{
					Triangle [i] [j] = inputTriangle [i] [j];
				}
			}
		}

		#endregion

		#region Public Methods

		public void SetVertexPosition(Vector3 v, int index)
		{
			if (VertexPosition != null && 
				VertexPosition.Length > index ) 
			{
				VertexPosition [index] = v;
			}
		}

		public void SetVertexPositions(Vector3[] v)
		{
			VertexPosition = new Vector3[v.Length];
			for (int i = 0; i < v.Length; i++)
				VertexPosition [i] = v[i];
		}

		public void SetAABB(AABB box)
		{
			AABBox = box;
		}
			
		#endregion

	}
}

