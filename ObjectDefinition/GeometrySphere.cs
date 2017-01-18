using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
    public class GeometrySphere
    {
        #region Fields

        public Vector3 Position { get; private set; }
        public IGeometry Geometry { get; private set; }


        #endregion

        #region Constructor

        public GeometrySphere() { }

        #endregion
    }
}
