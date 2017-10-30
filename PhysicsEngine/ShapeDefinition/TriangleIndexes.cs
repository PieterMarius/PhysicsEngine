using System;
using System.Linq;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public sealed class TriangleIndexes : IEquatable<TriangleIndexes>
    {
        #region Fields

        public readonly int a;
        public readonly int b;
        public readonly int c;

        #endregion

        #region Constructor

        public TriangleIndexes(int[] indexes)
        {
            if (indexes.Length < 3)
                throw new Exception("Wrong triangle indexes");

            a = indexes[0];
            b = indexes[1];
            c = indexes[2];
        }

        public TriangleIndexes(int a, int b, int c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        #endregion

        #region Public Methods

        public double this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return a;
                    case 1:
                        return b;
                    case 2:
                        return c;
                    default:
                        throw new ArgumentException("Wrong triangle index.", nameof(index));
                }
            }
        }

        public bool Contains(int index)
        {
            return a == index ||
                   b == index ||
                   c == index;
        }

        public static TriangleIndexes[] GenerateTriangleIndexes(int[][] trianglesIndexes)
        {
            TriangleIndexes[] result = new TriangleIndexes[trianglesIndexes.Length];

            foreach (var triangle in trianglesIndexes.Select((value, i) => new { value, i }))
            {
                result[triangle.i] = new TriangleIndexes(triangle.value);
            }

            return result;
        }

        #region IEquatable

        public override int GetHashCode()
        {
            return a.GetHashCode() ^
                   b.GetHashCode() ^
                   c.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return Equals(obj as TriangleIndexes);
        }

        public bool Equals(TriangleIndexes triangleIndexes)
        {
            return triangleIndexes != null &&
                   triangleIndexes.a == a &&
                   triangleIndexes.b == b &&
                   triangleIndexes.c == c;
        }

        #endregion

        #endregion

    }
}
