using System;

namespace SharpPhysicsEngine
{
    public sealed class HashGenerator
    {
        private Int32 Hash;

        public HashGenerator()
        {
            Hash = -1;
        }

        public Int32 GetHash()
        {
            Hash++;
            return Hash;
        }
    }
}
