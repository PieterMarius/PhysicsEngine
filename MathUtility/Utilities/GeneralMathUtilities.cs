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
using System.Threading;
using System.Threading.Tasks;

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

        public static double[] Multiply(
            double scalar, 
            double[] vector)
        {
            double[] result = new double[vector.Length];

            for (int i = 0; i < vector.Length; i++)
                result[i] = vector[i] * scalar;
            
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

        public static double Truncate(
            double value, 
            int precision)
        {
            var tPrecision = Math.Pow(10, precision);
            return Math.Truncate(value * tPrecision) / tPrecision;
        }

        public static Vector3 Truncate(
            Vector3 value,
            int precision)
        {
            var tPrecision = Math.Pow(10, precision);

            return new Vector3(
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

        public static double[][] GetArrayFromVector3(Vector3[] array)
        {
            var result = new double[array.Length][];

            for (int i = 0; i < array.Length; i++)
                result[i] = array[i].Array;

            return result;
        }

        public static double[][][] GetMatrixFromVector3Matrix(Vector3[][] matrix)
        {
            var result = new double[matrix.Length][][];

            for (int i = 0; i < matrix.Length; i++)
                result[i] = GetArrayFromVector3(matrix[i]);

            return result;
        }

        public static double[][] GetArrayFromVector2(Vector2[] array)
        {
            var result = new double[array.Length][];

            for (int i = 0; i < array.Length; i++)
                result[i] = array[i].Array;
            
            return result;
        }

        public static double[][][] GetMatrixFromVector2Matrix(Vector2[][] matrix)
        {
            var result = new double[matrix.Length][][];

            for (int i = 0; i < matrix.Length; i++)
                result[i] = GetArrayFromVector2(matrix[i]);
            
            return result;
        }
    }
}
