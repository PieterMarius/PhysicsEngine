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

namespace SharpEngineMathUtility
{
    public struct Vector2
    {
        #region Public Properties

        public readonly double x;
        public readonly double y;

        #endregion

        #region Constructors

        public Vector2(
            double x,
            double y)
        {
            this.x = x;
            this.y = y;
        }

        public Vector2(double[] vec)
        {
            if (vec.Length == 2)
            {
                x = vec[0];
                y = vec[1];
            }
            else
            {
                throw new ArgumentException(COMPONENT_EXCEPTION);
            }
        }

        public Vector2(Vector2 v)
        {
            x = v.x;
            y = v.y;
        }


        #endregion

        #region Public Methods

        public double[] Array
        {
            get
            {
                return new[] { x, y };
            }
        }

        #endregion

        #region Const

        private const string COMPONENT_EXCEPTION = "Vector must contain three components (x,y,z)";

        //TODO aggiungere eventuali altre eccezioni

        #endregion
    }
}
