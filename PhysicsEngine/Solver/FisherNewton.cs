using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.LCPSolver
{
    internal class FisherNewton : ISolver
    {

        public readonly SolverParameters SolverParameters;

        #region Constructor

        public FisherNewton(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            throw new NotImplementedException();
        }

        public double[] Solve(LinearProblemProperties linearProblemProperties, double[] x)
        {
            int N = x.Length;

            for (int i = 0; i < x.Length; i++)
            {

            }

            throw new NotImplementedException();

        }

        #endregion

        #region Private Methods



        #endregion
    }
}
