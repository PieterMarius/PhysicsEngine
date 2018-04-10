using System;

namespace SharpPhysicsEngine.Helper
{
    public class WarmStartHashSet: IEquatable<WarmStartHashSet>
    {
        public int ID_A { get; }
        public int ID_B { get; }

        public WarmStartHashSet(int id_a, int id_b)
        {
            this.ID_A = id_a;
            this.ID_B = id_b;
        }

        public override int GetHashCode()
        {
            return 31 * ID_A + 17 * ID_B;
        }

        public override bool Equals(object obj)
        {
            return obj is WarmStartHashSet && Equals((WarmStartHashSet)obj);
        }

        public bool Equals(WarmStartHashSet p)
        {
            return (ID_A == p.ID_A && ID_B == p.ID_B) ||
                   (ID_A == p.ID_B && ID_B == p.ID_A);
        }
    }
}
