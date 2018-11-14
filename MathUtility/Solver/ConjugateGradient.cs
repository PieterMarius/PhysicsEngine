using System;
using static SharpEngineMathUtility.MathUtils;
using static SharpEngineMathUtility.SparseMatrix;

namespace SharpEngineMathUtility.Solver
{
    public sealed class ConjugateGradient
    {

        #region Public Methods

        public double[] Solve(
            SparseMatrix A,
            double[] b,
            double[] x,
            int nIter,
            double tol = 1E-10)
        {
            double[] xi = x;
            double[] rNew = Minus(b, Multiply(A, x));
            double[] p = rNew;
            double r2Old = Dot(rNew, rNew);

            double alpha = 1.0;
            double beta = 1.0;

            for (int i = 0; i < nIter; i++)
            {
                alpha = GetAlpha(A, p, r2Old);

                xi = Add(xi, Multiply(alpha, p));

                rNew = Minus(rNew, Multiply(alpha, Multiply(A, p)));

                double r2New = Dot(rNew, rNew);

                if (r2New < tol)
                    return xi;

                beta = GetBeta(r2New, r2Old);

                p = Add(rNew, Multiply(beta, p));
                r2Old = r2New;
            }

            return xi;
        }

        #endregion

        #region Private Methods

        private double GetAlpha(
            SparseMatrix A,
            double[] p,
            double num)
        {
            var denom = Dot(p, Multiply(A, p));

            if (denom == 0.0)
                return 1.0;

            return num / denom;
        }
        public double GetBeta(
            double num,
            double denom)
        {
            if (denom == 0.0)
                return 1.0;

            return num / denom;
        }


        #endregion
    }
}
