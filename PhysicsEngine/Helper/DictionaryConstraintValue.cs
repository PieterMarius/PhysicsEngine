
namespace SharpPhysicsEngine.Helper
{
    public struct DictionaryConstraintValue
    {
        private readonly JacobianConstraint constraint;
        private readonly int index;

        public JacobianConstraint Constraint { get { return constraint; } }
        public int Index { get { return index; } }

        public DictionaryConstraintValue(
            JacobianConstraint constraint,
            int startIndex)
        {
            this.constraint = constraint;
            this.index = startIndex;
        }
    }
}
