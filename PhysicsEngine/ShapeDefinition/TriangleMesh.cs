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
using System.Linq;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class TriangleMesh : IEquatable<TriangleMesh>
    {
        #region Fields

        public readonly int a;
        public readonly int b;
        public readonly int c;

        #endregion

        #region Constructor

        public TriangleMesh(int[] indexes)
        {
            if (indexes.Length < 3)
                throw new Exception("Wrong triangle indexes");

            a = indexes[0];
            b = indexes[1];
            c = indexes[2];
        }

        public TriangleMesh(int a, int b, int c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        #endregion

        #region Public Methods

        public double this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return a;
                    case 1:
                        return b;
                    case 2:
                        return c;
                    default:
                        throw new ArgumentException("Wrong triangle index.", nameof(index));
                }
            }
        }

        public int[] GetArray()
        {
            return new int[] { a, b, c };
        }

        public bool Contains(int index)
        {
            return a == index ||
                   b == index ||
                   c == index;
        }

        public static TriangleMesh[] GenerateTriangleIndexes(int[][] trianglesIndexes)
        {
            TriangleMesh[] result = new TriangleMesh[trianglesIndexes.Length];

            foreach (var triangle in trianglesIndexes.Select((value, i) => new { value, i }))
            {
                result[triangle.i] = new TriangleMesh(triangle.value);
            }

            return result;
        }

        #region IEquatable

        public override int GetHashCode()
        {
            return a.GetHashCode() ^
                   b.GetHashCode() ^
                   c.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return Equals(obj as TriangleMesh);
        }

        public bool Equals(TriangleMesh triangleIndexes)
        {
            return triangleIndexes != null &&
                   triangleIndexes.a == a &&
                   triangleIndexes.b == b &&
                   triangleIndexes.c == c;
        }

        #endregion

        #endregion

    }
}
