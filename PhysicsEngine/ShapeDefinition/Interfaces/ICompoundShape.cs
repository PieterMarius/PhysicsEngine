using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public interface ICompoundShape
    {
        IGeometry[] ObjectGeometry { get; }
        double[] PartialMass { get; }
        Vector3[] StartCompoundPositionObjects { get; }
        int CompoundingConvexObjectCount { get; }

        void SetObjectGeometry(IGeometry[] geometry);
        void SetPartialMass(double[] mass);
        void SetCompoundPosition(Vector3[] compoundPosition);
    }
}
