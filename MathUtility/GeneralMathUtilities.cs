using System;

namespace SharpEngineMathUtility
{
    public static class GeneralMathUtilities
    {
        public static double Dot(double[] a, double[] b)
        {
            if (a.Length != b.Length)
                throw new Exception("Wrong vector length");

            double result = 0.0;

            for (int i = 0; i < a.Length; i++)
                result += a[i] * b[i];

            return result;
        }

        public static double[] Multiply(SparseElement[] matrix, double[] vector)
        {
            if(matrix[0].RowLength != vector.Length)
                throw new Exception("Wrong input length");

            double[] result = new double[matrix.Length];

            for (int i = 0; i < matrix.Length; i++)
            {
                SparseElement m = matrix[i];

                double[] bufValue = m.Value;
                int[] bufIndex = m.Index;

                double bValue = 0.0;

                for (int j = 0; j < m.Count; j++)
                    bValue += bufValue[j] * vector[bufIndex[j]];

                result[i] = bValue;
            }

            return result;
        }

        public static double[] Multiply(double scalar, double[] vector)
        {
            double[] result = new double[vector.Length];

            for (int i = 0; i < vector.Length; i++)
                result[i] = vector[i] * scalar;
            
            return result;
        }
        
        public static double[] Minus(double[] a, double[] b)
        {
            if (a.Length != b.Length)
                throw new Exception("Wrong vector length");

            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                result[i]= a[i] - b[i];

            return result;
        }

        public static double[] Plus(double[] a, double[] b)
        {
            if (a.Length != b.Length)
                throw new Exception("Wrong vector length");

            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                result[i] = a[i] + b[i];

            return result;
        }

        public static int SigBit(int x)
        {
            if (x < 0)
                return 0;
            --x;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

    }
}
