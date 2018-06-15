using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Solver
{
    internal sealed class NonLinearGaussSeidel
    {

        //      For n Newton iterations
        //        Update J, M and C using p
        //        For m Gauss-Seidel iterations
        //            For k constraints
        //                Solve lambda = -C / (J* M * J^ T)
        //                Update dp --- dp = M^-1 * J^T * lambda
        //                Update p  --- p+=dp
    }
}
