using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Solver
{
    internal enum FrictionStatus
    {
        None = 0,
        Gap = 1,
        SlipPositive = 2,
        SlipNegative = 3,
        Stick = 4,
    }
}
