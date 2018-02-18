using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Helper
{
    internal struct HashSetStruct : IEquatable<HashSetStruct>
    {
        private readonly int id_a, id_b;

        public int ID_A { get { return id_a; } }
        public int ID_B { get { return id_b; } }

        public HashSetStruct(int id_a, int id_b)
        {
            this.id_a = id_a;
            this.id_b = id_b;
        }

        public override int GetHashCode()
        {
            return 31 * id_a + 17 * id_b;
        }

        public override bool Equals(object obj)
        {
            return obj is HashSetStruct && Equals((HashSetStruct)obj);
        }

        public bool Equals(HashSetStruct p)
        {
            return id_a == p.ID_A && id_b == p.ID_B;
        }
    }
}
