using SharpEngineMathUtility;
using SharpEngineMathUtility.Solver;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static SharpEngineMathUtility.GeneralMathUtils;

namespace SharpPhysicsEngine.LCPSolver
{
    public class FisherNewton //: ISolver
    {

        #region Fields
               
        public readonly SolverParameters SolverParameters;

        private readonly GMRES solver;

        #endregion

        #region Constructor

        public FisherNewton(SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;
            solver = new GMRES();
        }

        #endregion

        #region Public Methods

        public SolverParameters GetSolverParameters()
        {
            throw new NotImplementedException();
        }

        //public double[] Solve(
        //    LinearProblemProperties linearProblemProperties,
        //    double[] x)
        //{
        //    return null;
        //}

        public double[] Solve(
            //LinearProblemProperties linearProblemProperties, 
            SparseElement[] A,
            double[] b,
            double[] x,
            double lambda)
        {
            int N = b.Length;
            //var A = linearProblemProperties.GetOriginalSparseMatrix();
            //var b = linearProblemProperties.B;
            double gamma = 1E-28;
            double error = double.PositiveInfinity;
            double oldError = 0.0;
            
            for (int iter = 0; iter < SolverParameters.MaxIteration; iter++)
            {
                double[] y = Add(SparseElement.Multiply(A, x, SolverParameters.MaxThreadNumber), b);
                double[] phi = PhiLambda(y, x, lambda);
                oldError = error;
                error = 0.5 * Dot(phi, phi);
                                
                var fnd = Find(phi, x, gamma);

                List<int> S = fnd.Item1;
                List<int> I = fnd.Item2;

                int restart = Math.Min(A.Length, 10);

                double[] nablaPhi = null;

                //TODO test other function
                var dx = RandomFunc(A, x, y, phi, I, N, restart, ref nablaPhi);
                                 
                x = ArmijoBacktracking(A, b, x, dx, nablaPhi, lambda, gamma, error);

            }

            throw new NotImplementedException();

        }

        #endregion

        #region Private Methods

        private double[] Fischer(double[] y, double[] x)
        {
            var y2 = Square(y);
            var x2 = Square(x);
            
            return Minus(Minus(SquareRoot(Add(y2, x2)), y), x);
        }

        private double[] PhiLambda(
            double[] a, 
            double[] b, 
            double lambda)
        {
            var t1 = Multiply(lambda, Fischer(a, b));
            var t2 = Multiply(1.0 - lambda, Multiply(Max(0, a), Max(0, b)));

            return Minus(t1, t2);
        }

        private double PsiLambda(double[] a, double[] b, double l)
        {
            var phi = PhiLambda(a, b, l);
            return 0.5 * Dot(phi, phi);
        }

        private Tuple<List<int>,List<int>> Find(
            double[] phi, 
            double[] x,
            double gamma)
        {
            var resT = new List<int>();
            var resF = new List<int>();

            for (int i = 0; i < phi.Length; i++)
            {
                if (Condition(phi[i], x[i], gamma))
                    resT.Add(i);
                else
                    resF.Add(i);
            }

            return new Tuple<List<int>, List<int>>(resT, resF);
        }
        
        private bool Condition(
            double phi, 
            double x,
            double gamma)
        {
            return Math.Abs(phi) < gamma && 
                   Math.Abs(x) < gamma;
        }

        private double[] RandomFunc(
            SparseElement[] A,
            double[] x,
            double[] y,
            double[] phi,
            List<int> I,
            int N,
            int restart,
            ref double[] nablaPhi)
        {
            var q = Minus(Multiply(1.0 / ConstValues.SqRoot2, GetRandom(0.0, 1.0, N)), 1.0);
            var p = Minus(Multiply(1.0 / ConstValues.SqRoot2, GetRandom(0.0, 1.0, N)), 1.0);
            var qi = GetQIndexTransform(x, y, q, I);
            var pi = GetPIndexTransform(x, y, p, I);
            var J = GetJacobianMatrix(A, pi, qi, I);
            var b = Multiply(-1.0, phi);
            nablaPhi = SparseElement.Multiply(J, phi);

            return solver.Solve(J, b, restart);
        }

        private double[] GetQIndexTransform(
            double[] x,
            double[] y,
            double[] q,
            List<int> I)
        {
            var res = new double[I.Count];
            Array.Copy(q, res, q.Length);

            for (int i = 0; i < I.Count; i++)
            {
                int idx = I[i];
                double by = y[idx];
                double bx = x[idx];
                double num = by;
                double den = Math.Sqrt(by * by + bx * bx);
                res[i] = num / den - 1.0;
            }

            return res;
        }

        private double[] GetPIndexTransform(
            double[] x,
            double[] y,
            double[] p,
            List<int> I)
        {
            var res = new double[p.Length];
            Array.Copy(p, res, p.Length);

            for (int i = 0; i < I.Count; i++)
            {
                int idx = I[i];
                double by = y[idx];
                double bx = x[idx];
                double num = bx;
                double den = Math.Sqrt(by * by + bx * bx);
                res[I[i]] = num / den - 1.0;
            }

            return res;
        }

        private SparseElement[] GetJacobianMatrix(
            SparseElement[] A,
            double[] pi,
            double[] qi,
            List<int> I)
        {
            return GetMatrixElements(A, qi, pi, I);
        }
        
        private SparseElement[] GetMatrixElements(
            SparseElement[] A,
            double[] mult,
            double[] add,
            List<int> I)
        {
            SparseElement[] res = new SparseElement[I.Count];

            for (int i = 0; i < I.Count; i++)
            {
                var bidx = I[i];
                var item = A[bidx];
                var index = -1;
                for (int j = 0; j < item.Index.Length; j++)
                {
                    if (item.Index[j] == bidx)
                    {
                        index = j;
                        break;
                    }
                }
                int[] idx = new int[] { bidx };
                double[] value = new double[] { item.Value[index] };
                value[0] *= mult[i];
                value[0] += add[i];

                res[i] = new SparseElement(value,idx, I.Count);
            }

            return res;
        }
        
        private double[] ArmijoBacktracking(
            SparseElement[] A,
            double[] b,
            double[] x,
            double[] dx,
            double[] nablaPhi,
            double lambda,
            double gamma,
            double err)
        {
            double tau = 1.0;
            double alpha = 0.5;
            double beta = 0.001;
            double f0 = err;
            double gradf = beta * Dot(nablaPhi, dx);
            double[] xk = new double[x.Length];
            Array.Copy(x, xk, x.Length);

            while(true)
            {
                xk = Max(0.0, Add(x, Multiply(tau, dx)));
                var yk = Add(SparseElement.Multiply(A, xk, SolverParameters.MaxThreadNumber), b);
                var phyk = PhiLambda(yk, xk, lambda);
                var fk = 0.5 * Dot(phyk, phyk);

                if (fk <= f0 + tau * gradf)
                    break;

                if (tau * tau < gamma)
                    break;

                tau = alpha * tau;
            }

            return xk;
        }

        #endregion
    }
}
