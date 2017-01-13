using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
    public class SoftShapePoint
    {
        #region Fields

        public Vector3 Position { get; private set; }
        //TODO: delete ???
        public Vector3 AngularVelocity { get; private set; }
        public Vector3 LinearVelocity { get; private set; }
        public double Diameter { get; private set; }
        
        #endregion

        #region Constructor

        public SoftShapePoint(double diameter)
        {
            Diameter = diameter;
        }

        #endregion

        #region Public Methods

        public void SetPosition(Vector3 position)
        {
            Position = position;
        }

        public void SetAngularVelocity(Vector3 angularVelocity)
        {
            AngularVelocity = angularVelocity;
        }

        public void SetLinearVelocity(Vector3 linearVelocity)
        {
            LinearVelocity = linearVelocity;
        }

        #endregion
    }
}
