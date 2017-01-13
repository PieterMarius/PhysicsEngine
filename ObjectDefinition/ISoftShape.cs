
namespace ShapeDefinition
{
    public interface ISoftShape
    {
        SoftShapePoint[] ShapePoint { get; }
        int[][] Triangle { get; }
        AABB AABBox { get; }

        void SetPointsMass(double mass);
        void SetShapePoint(SoftShapePoint[] shapePoint);
        void SetAABB();

    }
}
