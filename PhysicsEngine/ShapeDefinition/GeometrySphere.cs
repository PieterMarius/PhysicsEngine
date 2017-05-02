using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
    public class SoftPoint
    {
        #region Fields

        public Vector3 Position { get; private set; }
        public IGeometry Geometry { get; private set; }
        
        #endregion

        #region Constructor

        public SoftPoint(
            Vector3 position,
            IGeometry geometry)
        {
            Position = position;
            Geometry = geometry;
        }

        #endregion
    }
}
