using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LCPSolver
{
    public struct SolutionValues
    {
        public double X;
        public bool ConstraintStatus;

        public SolutionValues(double x, bool constraintStatus)
        {
            X = x;
            ConstraintStatus = constraintStatus;
        }
    }
}
