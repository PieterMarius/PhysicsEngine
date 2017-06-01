using SharpEngineMathUtility;
using System;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public static class Helper
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
