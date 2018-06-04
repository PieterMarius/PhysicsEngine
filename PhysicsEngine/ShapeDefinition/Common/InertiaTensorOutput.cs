using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class InertiaTensorOutput
    {
        #region Fields

        public Matrix3x3 InertiaTensor { get; private set; }
        public Vector3 CenterOfMass { get; private set; }
        public double DensityMass { get; private set; }

        #endregion

        #region Constructor

        public InertiaTensorOutput(
            Matrix3x3 inertiaTensor,
            Vector3 centerOfMass,
            double densityMass)
        {
            InertiaTensor = inertiaTensor;
            CenterOfMass = centerOfMass;
            DensityMass = densityMass;
        }

        #endregion
    }
}
