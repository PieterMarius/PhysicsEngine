using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class MassData
    {
        #region Fields

        public double Mass { get; set; }

        public double InverseMass { get; set; }

        public Matrix3x3 InertiaTensor { get; set; }

        public Matrix3x3 InverseInertiaTensor { get; set; }

        public Matrix3x3 InverseBaseInertiaTensor { get; set; }

        #endregion
    }
}
