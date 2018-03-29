using SharpEngineMathUtility;

namespace SharpPhysicsEngine.Helper
{
    internal class StabilizationValues
    {
        #region Public Fields

        public readonly Vector3 Position;
        public readonly Quaternion RotationStatus;
        public readonly Vector3 LinearVelocity;
        public readonly Vector3 AngularVelocity;

        #endregion

        #region Constructor

        public StabilizationValues(
            Vector3 position,
            Quaternion rotationStatus,
            Vector3 linearVelocity,
            Vector3 angularVelocity)
        {
            Position = position;
            RotationStatus = rotationStatus;
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
        }

        #endregion
    }
}
