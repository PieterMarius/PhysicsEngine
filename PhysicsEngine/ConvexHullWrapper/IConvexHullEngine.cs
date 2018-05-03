using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.ConvexHullWrapper
{
    internal interface IConvexHullEngine
    {
        TriangleMesh[] GetConvexHull(Vector3[] points);
    }
}