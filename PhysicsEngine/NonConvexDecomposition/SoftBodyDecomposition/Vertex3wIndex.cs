using ConvexHullGenerator;
using SharpEngineMathUtility;
using System.Collections.Generic;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public class Vertex3Index : IVertex
    {
        public Vertex3Index(Vector3 v, int[] indexes)
        {
            Vector3 = v;
            Indexes = indexes;
        }

        public Vector3 Vector3;
        public int[] Indexes;

        public double[] Position
        {
            get
            {
                return Vector3.Array;
            }
        }
    }
}
