using SharpEngineMathUtility;
using System;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal static class ShapeCommonUtilities
    {
        #region Public Methods

        public static Vector3 CalculateCenterOfMass(
            Vector3[] vertices,
            TriangleMesh[] triangleMeshes,
            double mass)
        {
            var inertiaTensor = new InertiaTensor(
                    vertices,
                    triangleMeshes,
                    mass,
                    false);

            return inertiaTensor.GetMassCenter();
        }

        public static Matrix3x3 GetInertiaTensor(
            VertexProperties[] vertex,
            TriangleMesh[] triangleMeshes,
            Vector3 position,
            double mass)
        {
            Matrix3x3 baseTensors = new Matrix3x3();

            Vector3[] vertexPosition = Array.ConvertAll(
                                    vertex,
                                    item => item.Vertex);

            var inertiaTensor = new InertiaTensor(
                    vertexPosition,
                    triangleMeshes,
                    mass,
                    true);

            var normalizedInertiaTensor = inertiaTensor;

            Vector3 r = inertiaTensor.GetMassCenter() - position;
            baseTensors += inertiaTensor.GetInertiaTensor() +
                            (Matrix3x3.IdentityMatrix() * r.Dot(r) - Matrix3x3.OuterProduct(r, r)) *
                            mass;

            return baseTensors;
        }

        #endregion

        #region Private Methods

        

        #endregion
    }
}
