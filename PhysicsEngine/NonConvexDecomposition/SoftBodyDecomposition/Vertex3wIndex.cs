using ConvexHullGenerator;
using SharpEngineMathUtility;
using System.Linq;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public class Vertex3Index : IVertex
    {
        public Vertex3Index(Vector3 v, int[] indexes)
        {
            Vector3 = v;
            Indexes = indexes;
        }

        public Vector3 Vector3 { get; private set; }
        public int[] Indexes { get; private set; }

        public double[] Position
        {
            get
            {
                return Vector3.Array;
            }
        }
    }
}
