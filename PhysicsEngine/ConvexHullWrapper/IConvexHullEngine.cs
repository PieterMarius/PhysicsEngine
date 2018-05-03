using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ConvexHullWrapper
{
    internal interface IConvexHullEngine
    {
        ConvexHullData GetConvexHull(Vector3[] points);
    }
}