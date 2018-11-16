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
using System.Collections.Generic;

namespace SharpEngineMathUtility
{
    public struct SparseVector : IEqualityComparer<SparseVector>
    {
        #region Fields

        public double[] Value { get; private set; }
        public int[] Index { get; private set; }
        public int Length;

        private Dictionary<int, double> _elements;
        public Dictionary<int, double> Elements 
        { 
            get
            {
                if(_elements == null)
                {
                    _elements = GetDictionary();
                }
                return _elements;
            }
        }
		
		#endregion

		#region Constructor

		public SparseVector (
			double[] value,
			int[] index,
            int length)
		{
			Value = value;
			Index = index;
			Length = length;
            _elements = null;
		}

        #endregion

        #region Public Methods
                
        public bool Equals(SparseVector x, SparseVector y)
        {
            if (x.Length != y.Length)
                return false;

            if (x.Index.Length != x.Index.Length)
                return false;

            if (x.Value.Length != x.Value.Length)
                return false;
                        
            return true;
        }

        public int GetHashCode(SparseVector obj)
        {
            throw new NotImplementedException();
        }

        public double[] GetArray()
        {
            double[] result = new double[Length];

            for (int i = 0; i < Index.Length; i++)
                result[Index[i]] = Value[i]; 
            
            return result;
        }

        public static double[] GetArray(SparseVector v)
        {
            double[] result = new double[v.Length];

            for (int i = 0; i < v.Index.Length; i++)
                result[v.Index[i]] = v.Value[i];
            
            return result;
        }

        public static SparseVector GetSparseElement(double[] v, double tol = 1E-50)
        {
            List<int> index = new List<int>();
            List<double> value = new List<double>();
            
            for (int i = 0; i < v.Length; i++)
            {
                var item = v[i];
                if(Math.Abs(item) > tol)
                {
                    index.Add(i);
                    value.Add(item);
                }
            }

            return new SparseVector(value.ToArray(), index.ToArray(), v.Length);
        }

        

        public HashSet<int> GetIndexHashset()
        {
            return new HashSet<int>(Index);
        }
               
        #endregion

        #region Private Methods
               
        private Dictionary<int, double> GetDictionary()
        {
            var res = new Dictionary<int, double>();

            for (int i = 0; i < Index.Length; i++)
                res.Add(Index[i], Value[i]);
            
            return res;
        }

        #endregion

    }
}

