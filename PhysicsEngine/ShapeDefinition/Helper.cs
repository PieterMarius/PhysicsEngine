﻿using PhysicsEngineMathUtility;
using System;

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

        public static AABB UpdateAABB(SoftShapePoint[] shapePoint)
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

        public static Vector3 GetVertexPosition(
            IGeometry obj,
            int vertexIndex)
        {
            return
                obj.Shape.Position +
                (obj.Shape.RotationMatrix * obj.RelativePosition[vertexIndex]);
        }


        public static IGeometry[] GetGeometry(IShape shape)
        {
            ICompoundShape compoundShape = shape as ICompoundShape;
            if (compoundShape != null)
                return compoundShape.ObjectGeometry;

            IConvexShape convexShape = shape as IConvexShape;
            if (convexShape != null)
                return new IGeometry[] { convexShape.ObjectGeometry };
            
            throw new ArgumentException("Unexpected type: " + shape.GetType());
        }
    }
}
