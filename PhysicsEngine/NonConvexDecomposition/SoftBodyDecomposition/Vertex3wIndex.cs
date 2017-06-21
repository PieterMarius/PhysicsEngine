using SharpEngineMathUtility;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public class Vertex3Index
    {
        public Vertex3Index(Vector3 v, int[] indexes)
        {
            Vector3 = v;
            Indexes = indexes;
        }

        public Vector3 Vector3 { get; private set; }
        public int[] Indexes { get; private set; }
    }
}
