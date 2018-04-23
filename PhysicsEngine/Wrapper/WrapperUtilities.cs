using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper
{
    internal static class WrapperUtilities
    {
        public static TriangleMesh[] GetTriangleMeshes(int[][] inputTriangle)
        {
            TriangleMesh[] triangleMeshes = new TriangleMesh[inputTriangle.Length];

            for (int i = 0; i < inputTriangle.Length; i++)
                triangleMeshes[i] = new TriangleMesh(
                    inputTriangle[i][0],
                    inputTriangle[i][1],
                    inputTriangle[i][2]);

            return triangleMeshes;
        }
    }
}
