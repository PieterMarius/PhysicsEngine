using SharpEngineMathUtility;
using System;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal static class Helper
    {
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
            if (shape is ICompoundShape compoundShape)
                return compoundShape.ObjectGeometry;

            if (shape is IConvexShape convexShape)
                return new IGeometry[] { convexShape.ObjectGeometry };

            throw new ArgumentException("Unexpected type: " + shape.GetType());
        }
    }
}
