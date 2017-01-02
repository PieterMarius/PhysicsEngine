using PhysicsEngineMathUtility;
using SimulationObjectDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SimulationObjectDefinition
{
    public static class Helper
    {
        public static AABB UpdateAABB(
            SimulationObject simObject,
            int geometryIndex)
        {
            Vector3 vertexPos = GetVertexPosition(simObject, geometryIndex, 0);
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < simObject.RelativePositions[geometryIndex].Length; i++)
            {
                Vector3 vertex = GetVertexPosition(simObject, geometryIndex, i);

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
            SimulationObject obj,
            int geometryIndex,
            int vertexIndex)
        {
            return
                obj.Position +
                (obj.RotationMatrix * obj.RelativePositions[geometryIndex][vertexIndex]);
        }
    }
}
