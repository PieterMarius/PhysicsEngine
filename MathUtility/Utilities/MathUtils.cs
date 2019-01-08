/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;

namespace SharpEngineMathUtility
{
    public static class MathUtils
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

        public static double Module(double[] a)
        {
            if (a == null || a.Length == 0)
                throw new Exception("Wrong vector length");

            double result = 0.0;

            for (int i = 0; i < a.Length; i++)
                result += a[i] * a[i];

            return Math.Sqrt(result);
        }

        public static double[] Normalize(double[] a)
        {
            if (a == null || a.Length == 0)
                throw new Exception("Wrong vector length");

            double[] result = new double[a.Length];
            double module = 1.0 / Module(a);

            for (int i = 0; i < a.Length; i++)
                result[i]= a[i] *module;

            return result;
        }

        public static double[] Multiply(
            double scalar, 
            double[] vector)
        {
            double[] result = new double[vector.Length];

            for (int i = 0; i < vector.Length; i++)
                result[i] = vector[i] * scalar;
            
            return result;
        }

        public static double[] Multiply(
            double scalar,
            double[] vector,
            int limit)
        {
            double[] result = new double[limit];

            for (int i = 0; i < limit; i++)
                result[i] = vector[i] * scalar;

            return result;
        }

        public static double[] Multiply(double[][] m, double[] v, int length)
        {
            double[] result = new double[m.Length];
            for (int i = 0; i < m.Length; i++)
            {
                double temp = 0.0;
                for  (int j = 0; j < length; j++)
                {
                    temp += m[i][j] * v[j];
                }
                result[i] = temp;
            }

            return result;
        }

        public static double[] Div(double[][] m, double[] v, int length)
        {
            double[] result = new double[length];
            for (int i = 0; i < length; i++)
            {
                double temp = 0.0;
                for (int j = 0; j < length; j++)
                {
                    temp += m[i][j] / v[j];
                }
                result[i] = temp;
            }

            return result;
        }

        public static double[] Multiply(double[][] m, double[] v)
        {
            double[] result = new double[m.Length];
            for (int i = 0; i < v.Length; i++)
            {
                double temp = 0.0;
                for (int j = 0; j < m[i].Length; j++)
                {
                    temp += m[i][j] * v[j];
                }
                result[i] = temp;
            }

            return result;
        }

        public static double[] ParallelMultiply(
            double scalar,
            double[] vector,
            int maxThread)
        {
            double[] result = new double[vector.Length];
            
            var rangePartitioner = Partitioner.Create(0, vector.Length, Convert.ToInt32(vector.Length / maxThread) + 1);

            Parallel.ForEach(
                rangePartitioner,
                new ParallelOptions() { MaxDegreeOfParallelism = maxThread },
                (range, loopState) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                        result[i] = vector[i] * scalar;
                });

            return result;
        }

        public static double[] Minus(double[] a, double[] b, bool simd = false)
        {
            if (a.Length != b.Length)
                throw new Exception("Wrong vector length");

            if (simd)
            {
                return SIMDUtils.SIMDArraySubtraction(a, b);
            }
            else
            {
                double[] result = new double[a.Length];

                for (int i = 0; i < a.Length; i++)
                    result[i] = a[i] - b[i];

                return result;
            }
        }

        public static double[] Minus(double[] a, double b)
        {
            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                result[i] = a[i] - b;

            return result;
        }

        public static double[] Add(double[] a, double[] b)
        {
            if (a.Length != b.Length)
                throw new Exception("Wrong vector length");

            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                result[i] = a[i] + b[i];

            return result;
        }

        public static double[] Add(double[] a, double b)
        {
            double[] result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                result[i] += a[i] + b;

            return result;
        }

        public static double[] Square(double[] a)
        {
            double[] res = new double[a.Length];
            for (int i = 0; i < a.Length; i++)
            {
                double val = a[i];
                res[i] = val * val;
            }

            return res;
        }

        public static double[] Multiply(double[] a, double[] b)
        {
            if (a.Length != b.Length)
                throw new Exception();

            var res = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
                res[i] = a[i] * b[i];

            return res;
        }

        public static double[] Max(double min, double[] v)
        {
            var res = new double[v.Length];

            for (int i = 0; i < v.Length; i++)
                res[i] = Math.Max(min, v[i]);

            return res;
        }

        public static double Max(double[] v, ref int idx)
        {
            var max = double.NegativeInfinity;

            for (int i = 0; i < v.Length; i++)
            {
                if (v[i] > max)
                {
                    max = v[i];
                    idx = i;
                }
            }
            
            return max;
        }

        public static double Max(double[] v)
        {
            var max = double.NegativeInfinity;

            for (int i = 0; i < v.Length; i++)
            {
                if (v[i] > max)
                    max = v[i];
            }

            return max;
        }

        public static double Min(double[] v, ref int idx)
        {
            var min = double.PositiveInfinity;

            for (int i = 0; i < v.Length; i++)
            {
                if (v[i] < min)
                {
                    min = v[i];
                    idx = i;
                }
            }

            return min;
        }

        public static double Min(double[] v)
        {
            var min = double.PositiveInfinity;

            for (int i = 0; i < v.Length; i++)
            {
                if (v[i] < min)
                    min = v[i];
            }

            return min;
        }

        public static double[] SquareRoot(double[] a)
        {
            double[] res = new double[a.Length];
            for (int i = 0; i < a.Length; i++)
                res[i] = Math.Sqrt(a[i]);
            
            return res;
        }

        public static double Truncate(
            double value, 
            int precision)
        {
            var tPrecision = Math.Pow(10, precision);
            return Math.Truncate(value * tPrecision) / tPrecision;
        }

        public static Vector3d Truncate(
            Vector3d value,
            int precision)
        {
            var tPrecision = Math.Pow(10, precision);

            return new Vector3d(
                Math.Truncate(value.x * tPrecision) / tPrecision,
                Math.Truncate(value.y * tPrecision) / tPrecision,
                Math.Truncate(value.z * tPrecision) / tPrecision);
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

        public static double Add(ref double location1, double value)
        {
            double newCurrentValue = location1; // non-volatile read, so may be stale
            while (true)
            {
                double currentValue = newCurrentValue;
                double newValue = currentValue + value;
                newCurrentValue = Interlocked.CompareExchange(ref location1, newValue, currentValue);
                if (newCurrentValue == currentValue)
                    return newValue;
            }
        }

        public static double[][] GetArrayFromVector3(Vector3d[] array)
        {
            var result = new double[array.Length][];

            for (int i = 0; i < array.Length; i++)
                result[i] = array[i].Array;

            return result;
        }

        public static double[][][] GetMatrixFromVector3Matrix(Vector3d[][] matrix)
        {
            var result = new double[matrix.Length][][];

            for (int i = 0; i < matrix.Length; i++)
                result[i] = GetArrayFromVector3(matrix[i]);

            return result;
        }

        public static double[][] GetArrayFromVector2(Vector2d[] array)
        {
            var result = new double[array.Length][];

            for (int i = 0; i < array.Length; i++)
                result[i] = array[i].Array;
            
            return result;
        }

        public static double[][][] GetMatrixFromVector2Matrix(Vector2d[][] matrix)
        {
            var result = new double[matrix.Length][][];

            for (int i = 0; i < matrix.Length; i++)
                result[i] = GetArrayFromVector2(matrix[i]);
            
            return result;
        }

        public static Vector3d[] GetVector3ArrayFromMatrix(double[][] input)
        {
            var result = new Vector3d[input.Length];

            for (int i = 0; i < input.Length; i++)
            {
                result[i] = new Vector3d(input[i]);
            }

            return result;
        }

        public static T[] FlattenArray<T>(T[][] matrix)
        {
            var rows = matrix.Length;
            var list1d = new List<T>();

            for (int i = 0; i < rows; i++)
            {
                int cols = matrix[i].Length;
                for (int j = 0; j < cols; j++)
                {
                    list1d.Add(matrix[i][j]);
                }
            }

            return list1d.ToArray();
        }

        public static T[] GetRow<T>(this T[,] array, int row)
        {
            if (!typeof(T).IsPrimitive)
                throw new InvalidOperationException("Not supported for managed types.");

            if (array == null)
                throw new ArgumentNullException("array");

            int cols = array.GetUpperBound(1) + 1;
            T[] result = new T[cols];
            int size = Marshal.SizeOf<T>();

            Buffer.BlockCopy(array, row * cols * size, result, 0, cols * size);

            return result;
        }

        public static double[] GetRandom(
            double min,
            double max,
            int length)
        {
            var res = new double[length];
            for (int i = 0; i < length; i++)
            {
                res[i] = GeometryUtils.GetRandom(0.0, 1.0);
            }

            return res;
        }
    }
}
