using SharpEngineMathUtility;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;

namespace SharpPhysicsEngine.ShapeDefinition
{
	public class Geometry : IGeometry
	{
		#region Object Properties

		/// <summary>
		/// Vertex Position
		/// </summary>
		public VertexProperties[] VertexPosition { get; private set; }
		/// <summary>
		/// Triangle Index
		/// </summary>
		public TriangleIndexes[] Triangle { get; private set; }
		/// <summary>
		/// Bounding Box
		/// </summary>
		public AABB AABBox { get; private set; }
		/// <summary>
		/// Get the geometry property of object
		/// </summary>
		public ObjectGeometryType GeometryType { get; private set; }
		/// <summary>
		/// Relative position respect mass center
		/// </summary>
		public Vector3[] RelativePosition { get; private set; }
		/// <summary>
		/// Pointer to belonging shape 
		/// </summary>
		public IShape Shape { get; private set; }

		#endregion

		#region Constructor

		public Geometry (
			IShape shape,
			Vector3[] inputVertexPosition,
			TriangleIndexes[] inputTriangle,
			ObjectGeometryType geometryType,
			bool getAdjacencyList)
		{
			Shape = shape;
			GeometryType = geometryType;

			if (inputTriangle != null)
			{
				Triangle = inputTriangle;
				
				SetVertexAdjacency(inputVertexPosition, getAdjacencyList);
			}
			else
			{
				SetVertexAdjacency(inputVertexPosition, getAdjacencyList);
			}
		}

		public Geometry(
			IShape shape,
			Vector3[] inputVertexPosition,
			ObjectGeometryType geometryType)
			: this(shape, inputVertexPosition, null, geometryType, false)
		{ }

		#endregion

			#region Public Methods

		public void SetVertexPosition(Vector3 v, int index)
		{
			if (VertexPosition != null && 
				VertexPosition.Length > index ) 
			{
				VertexPosition[index].SetVertexPosition(v);
			}
		}

		public void SetVertexPositions(Vector3[] v)
		{
			for (int i = 0; i < v.Length; i++)
				VertexPosition [i].SetVertexPosition(v[i]);
		}

		public void SetRelativePosition(Vector3[] relativePosition)
		{
			RelativePosition = relativePosition;
		}

		public void SetAABB(AABB box)
		{
			AABBox = box;
		}

		public void SetShape(IShape shape)
		{
			Shape = shape;
		}

		#endregion

		#region Private Methods

		private void SetVertexAdjacency(
			Vector3[] inputVertexPosition,
			bool getAdjacencyList)
		{
			VertexPosition = new VertexProperties[inputVertexPosition.Length];

			Parallel.For(0,
				VertexPosition.Length,
				new ParallelOptions { MaxDegreeOfParallelism = 4 },
				i =>
				{
					VertexPosition[i] = new VertexProperties(inputVertexPosition[i]);

					if (Triangle != null && 
						getAdjacencyList)
					{
						HashSet<int> adjacencyList = new HashSet<int>();

						for (int j = 0; j < Triangle.Length; j++)
						{
							if (Triangle[j].a == i)
							{
								adjacencyList.Add(Triangle[j].b);
								adjacencyList.Add(Triangle[j].c);
								continue;
							}
							if (Triangle[j].b == i)
							{
								adjacencyList.Add(Triangle[j].a);
								adjacencyList.Add(Triangle[j].c);
								continue;
							}
							if (Triangle[j].c == i)
							{
								adjacencyList.Add(Triangle[j].a);
								adjacencyList.Add(Triangle[j].b);
								continue;
							}
						}
						VertexPosition[i].SetAdjacencyList(adjacencyList.ToList());
					}
				});
		}

		#endregion
	}
}

