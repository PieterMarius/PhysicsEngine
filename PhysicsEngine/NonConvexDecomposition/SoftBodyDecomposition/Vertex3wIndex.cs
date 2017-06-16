using SharpEngineMathUtility;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public struct Vertex3wIndex 
    {
        public Vertex3wIndex(Vector3 v, int i)
        {
            vector3 = v;
            index = i;
        }

        public Vector3 vector3;
        public int index;
    }
}
