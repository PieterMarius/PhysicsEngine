﻿/******************************************************************************
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

using SharpEngineMathUtility;
using System;
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    internal sealed class Vertex3Index
    {
        #region Fields

        public Vector3d Vector3 { get; private set; }
        public HashSet<int> Indexes { get; private set; }
        public int ID { get; private set; }

        #endregion

        #region Constructor

        public Vertex3Index(
            Vector3d v, 
            HashSet<int> index,
            int id)
        {
            Vector3 = v;
            Indexes = index;
            ID = id;
        }

        #endregion

        #region Public Methods

        public void AddIndex(int index)
        {
            Indexes.Add(index);
        }

        #endregion

    }
}
