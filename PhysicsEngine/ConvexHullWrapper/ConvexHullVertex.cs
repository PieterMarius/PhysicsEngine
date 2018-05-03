using MIConvexHull;

namespace SharpPhysicsEngine.ConvexHullWrapper
{
    internal sealed class ConvexHullVertex : IVertex
    {
        public double[] Position { get; set; }

        public int Index { get; set; }
    }
}
