
namespace SharpPhysicsEngine.ShapeDefinition
{
    public interface IConvexShape
    {
        IGeometry ObjectGeometry { get; }
        
        void SetObjectGeometry(IGeometry geometry);
        
        
    }
}
