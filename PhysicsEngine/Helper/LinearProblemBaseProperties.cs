using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Helper
{
    internal struct LinearProblemBaseProperties
    {
        #region Fields

        public double[] B;
        public double[] D;
        public double[] InvD;
        public ConstraintType[] ConstraintType;
        public double[] ConstraintLimit;
        public SparseElement[] M;
        public int?[] ConstraintsArray;

        #endregion

        #region Constructor

        public LinearProblemBaseProperties(
            int constraintLength)
        {
            B = new double[constraintLength];
            D = new double[constraintLength];
            InvD = new double[constraintLength];
            ConstraintType = new ConstraintType[constraintLength];
            ConstraintLimit = new double[constraintLength];
            M = new SparseElement[constraintLength];
            ConstraintsArray = new int?[constraintLength];
        }

        #endregion
    }
}
