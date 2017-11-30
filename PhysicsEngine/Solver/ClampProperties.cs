
namespace SharpPhysicsEngine.Solver
{
    internal class ClampProperties
    {
        #region Public Properties

        public double Value { get; private set; }
        public FrictionStatus FrictionStatus { get; private set; }

        #endregion

        #region Constructor

        public ClampProperties(
            double value,
            FrictionStatus frictionStatus)
        {
            Value = value;
            FrictionStatus = frictionStatus;
        }

        #endregion
    }
}
