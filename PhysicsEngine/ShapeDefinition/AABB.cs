
using SharpEngineMathUtility;
using SharpPhysicsEngine.NonConvexDecomposition.Octree;

namespace SharpPhysicsEngine.ShapeDefinition
{
	public class AABB
	{

		#region Object fields

		public Vector3 Min;
		public Vector3 Max;
		public bool positionAABBChanged { get; private set; }

		#endregion

		#region Constructor

		public AABB (
			double minX,
			double maxX,
			double minY,
			double maxY,
			double minZ,
			double maxZ,
			bool positionChanged)
		{
			Min = new Vector3(minX, minY, minZ);
            Max = new Vector3(maxX, maxY, maxZ);
			positionAABBChanged = positionChanged;
		}

        public AABB(Vector3 min, Vector3 max)
        {
            Min = min;
            Max = max;
        }

        #endregion

        #region Public methods

        public void SetPositionChanged(bool value)
		{
			positionAABBChanged = value; 
		}

        public bool Contains(AABB box)
        {
            return Min.x <= box.Min.x && Max.x >= box.Max.x &&
                Min.y <= box.Min.y && Max.y >= box.Max.y &&
                Min.z <= box.Min.z && Max.z >= box.Max.z;
        }

        public bool Intersect(AABB box)
        {
            return Min.x > box.Max.x || Max.x < box.Min.x ||
                Min.y > box.Max.y || Max.y < box.Min.y ||
                Min.z > box.Max.z || Max.z < box.Min.z;
        }

        #endregion

        #region Public static methods

        public static AABB GetGeometryAABB(IGeometry simObject)
        {
            Vector3 vertexPos = Helper.GetVertexPosition(simObject, 0);
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < simObject.RelativePosition.Length; i++)
            {
                Vector3 vertex = Helper.GetVertexPosition(simObject, i);

                if (vertex.x < xMin)
                    xMin = vertex.x;
                else if (vertex.x > xMax)
                    xMax = vertex.x;

                if (vertex.y < yMin)
                    yMin = vertex.y;
                else if (vertex.y > yMax)
                    yMax = vertex.y;

                if (vertex.z < zMin)
                    zMin = vertex.z;
                else if (vertex.z > zMax)
                    zMax = vertex.z;
            }

            return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
        }

        public static AABB GetShapePointAABB(SoftShapePoint[] shapePoint)
        {
            Vector3 vertexPos = shapePoint[0].Position;
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < shapePoint.Length; i++)
            {
                Vector3 vertex = shapePoint[i].Position;

                if (vertex.x < xMin)
                    xMin = vertex.x;
                else if (vertex.x > xMax)
                    xMax = vertex.x;

                if (vertex.y < yMin)
                    yMin = vertex.y;
                else if (vertex.y > yMax)
                    yMax = vertex.y;

                if (vertex.z < zMin)
                    zMin = vertex.z;
                else if (vertex.z > zMax)
                    zMax = vertex.z;
            }

            return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
        }

        public static AABB GetTriangleAABB(Vector3[] triangle)
        {
            Vector3 vertexPos = triangle[0];
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < triangle.Length; i++)
            {
                Vector3 vertex = triangle[i];

                if (vertex.x < xMin)
                    xMin = vertex.x;
                else if (vertex.x > xMax)
                    xMax = vertex.x;

                if (vertex.y < yMin)
                    yMin = vertex.y;
                else if (vertex.y > yMax)
                    yMax = vertex.y;

                if (vertex.z < zMin)
                    zMin = vertex.z;
                else if (vertex.z > zMax)
                    zMax = vertex.z;
            }

            return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
        }

        #endregion
    }
}

