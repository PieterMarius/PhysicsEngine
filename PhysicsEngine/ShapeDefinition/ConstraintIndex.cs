using System;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public sealed class ConstraintIndex : IEquatable<ConstraintIndex>
    {

        #region Fields

        public int IndexA { get; private set; }
        public int IndexB { get; private set; }

        #endregion

        #region Constructor

        public ConstraintIndex(
            int indexA,
            int indexB)
        {
            IndexA = indexA;
            IndexB = indexB;
        }

        #endregion

        #region Public Methods

        public override int GetHashCode()
        {
            return IndexA.GetHashCode() ^
                   IndexB.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return Equals(obj as ConstraintIndex);
        }

        public bool Equals(ConstraintIndex constraintIndex)
        {
            return constraintIndex != null &&
                   (constraintIndex.IndexA.Equals(IndexA) &&
                   constraintIndex.IndexB.Equals(IndexB)) ||
                   (constraintIndex.IndexB.Equals(IndexA) &&
                   constraintIndex.IndexA.Equals(IndexB));
        }

        #endregion
    }
}
