
namespace ShapeDefinition
{
    public interface ISoftShape
    {
        SoftShapePoint[] ShapePoint { get; }
        int[][] Triangle { get; }
        AABB AABBox { get; }
        //TODO add constraint connector
        GeometrySphere[] Sphere { get; }

        void SetPointsMass(double mass);
        void SetShapePoint(SoftShapePoint[] shapePoint);
        void SetAABB();
        void SetGeometrySphere(GeometrySphere[] geometrySphere);

    }
}
