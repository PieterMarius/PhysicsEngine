using System.Numerics;

namespace SharpEngineMathUtility
{
    public static class SIMDMathUtility
    {

        public static double[] SIMDArrayAddition(double[] lhs, double[] rhs)
        {
            var simdLength = Vector<double>.Count;
            var result = new double[lhs.Length];
            var i = 0;
            for (i = 0; i <= lhs.Length - simdLength; i += simdLength)
            {
                var va = new Vector<double>(lhs, i);
                var vb = new Vector<double>(rhs, i);
                (va + vb).CopyTo(result, i);
            }

            for (; i < lhs.Length; ++i)
            {
                result[i] = lhs[i] + rhs[i];
            }

            return result;
        }

        public static double[] SIMDArrayProduct(double[] lhs, double[] rhs)
        {
            var simdLength = Vector<double>.Count;
            var result = new double[lhs.Length];
            var i = 0;
            for (i = 0; i <= lhs.Length - simdLength; i += simdLength)
            {
                var va = new Vector<double>(lhs, i);
                var vb = new Vector<double>(rhs, i);
                (va * vb).CopyTo(result, i);
            }

            for (; i < lhs.Length; ++i)
            {
                result[i] = lhs[i] * rhs[i];
            }

            return result;
        }

        public static double[] SIMDArrayProductScalar(double[] lhs, double scalar)
        {
            var simdLength = Vector<double>.Count;
            var result = new double[lhs.Length];
            var i = 0;
            for (i = 0; i <= lhs.Length - simdLength; i += simdLength)
            {
                var va = new Vector<double>(lhs, i);
                (va * scalar).CopyTo(result, i);
            }

            for (; i < lhs.Length; ++i)
            {
                result[i] = lhs[i] * scalar;
            }

            return result;
        }
    }
}
