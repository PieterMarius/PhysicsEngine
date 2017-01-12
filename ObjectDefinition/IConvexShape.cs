
namespace ShapeDefinition
{
    public interface IConvexShape
    {
        IGeometry ObjectGeometry { get; }
        
        void SetObjectGeometry(IGeometry geometry);
        void SetMass(double mass);
        
    }
}
