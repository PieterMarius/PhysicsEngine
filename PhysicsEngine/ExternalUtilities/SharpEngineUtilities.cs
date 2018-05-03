using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.Wrapper;

namespace SharpPhysicsEngine.ExternalUtilities
{
    public static class SharpEngineUtilities
    {
        public static Vector3 GetCenterOfMass(
            Vector3[] vertices,
            int[][] triangleMeshes,
            double mass)
        {
            var triMeshes = WrapperUtilities.GetTriangleMeshes(triangleMeshes);

            return ShapeCommonUtilities.CalculateCenterOfMass(
                vertices,
                triMeshes,
                mass);
        }


    }
}
