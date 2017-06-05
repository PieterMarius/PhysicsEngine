﻿using System;
using System.Linq;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public sealed class TriangleIndexes
    {
        #region Fields

        public readonly int a;
        public readonly int b;
        public readonly int c;

        #endregion

        #region Constructor

        public TriangleIndexes(int[] indexes)
        {
            if (indexes.Length < 3)
                throw new Exception("Wrong triangle indexes");

            a = indexes[0];
            b = indexes[1];
            c = indexes[2];
        }

        public TriangleIndexes(int a, int b, int c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        #endregion

        #region Public Methods

        public bool Contains(int index)
        {
            return a == index ||
                   b == index ||
                   c == index;
        }

        public static TriangleIndexes[] GenerateTriangleIndexes(int[][] trianglesIndexes)
        {
            TriangleIndexes[] result = new TriangleIndexes[trianglesIndexes.Length];

            foreach (var triangle in trianglesIndexes.Select((value, i) => new { value, i }))
            {
                result[triangle.i] = new TriangleIndexes(triangle.value);
            }

            return result;
        }

        #endregion

    }
}
