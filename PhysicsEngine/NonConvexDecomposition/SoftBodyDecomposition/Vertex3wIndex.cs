using SharpEngineMathUtility;
using System;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public sealed class Vertex3Index: IEquatable<Vertex3Index>
    {
        #region Fields

        public Vector3 Vector3 { get; private set; }
        public int[] Indexes { get; private set; }
        public int ID { get; private set; }

        #endregion

        #region Constructor

        public Vertex3Index(
            Vector3 v, 
            int[] index,
            int id)
        {
            Vector3 = v;
            Indexes = index;
            ID = id;
        }

        public override int GetHashCode()
        {
            return Indexes.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return Equals(obj as Vertex3Index);
        }

        public bool Equals(Vertex3Index other)
        {
            if (Indexes.Length != other.Indexes.Length)
                return false;

            for (int i = 0; i < Indexes.Length; i++)
            {
                if (Indexes[i] != other.Indexes[i])
                    return false;
            }
           
            return true;
        }

        #endregion

    }
}
