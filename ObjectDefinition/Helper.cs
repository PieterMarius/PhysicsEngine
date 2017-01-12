using PhysicsEngineMathUtility;
using ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeDefinition
{
    public static class Helper
    {
        public static AABB UpdateAABB(IGeometry simObject)
        {
            Vector3 vertexPos = GetVertexPosition(simObject, 0);
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < simObject.RelativePosition.Length; i++)
            {
                Vector3 vertex = GetVertexPosition(simObject, i);

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

        public static Vector3 GetVertexPosition(
            IGeometry obj,
            int vertexIndex)
        {
            return
                obj.Shape.Position +
                (obj.Shape.RotationMatrix * obj.RelativePosition[vertexIndex]);
        }


        public static IGeometry[] GetGeometry(
            IShape shape)
        {
            if (shape is ICompoundShape)
                return ((ICompoundShape)shape).ObjectGeometry;
            else if (shape is IConvexShape)
                return new IGeometry[] { ((IConvexShape)shape).ObjectGeometry };
            else
                throw new ArgumentException("Unexpected type: " + shape.GetType());
        }
    }
}
