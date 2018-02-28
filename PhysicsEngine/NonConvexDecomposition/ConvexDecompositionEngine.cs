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

using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpPhysicsEngine.Wrapper;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.NonConvexDecomposition
{
    public sealed class ConvexDecompositionEngine
    {
        #region Private Properties

        ShapeConvexDecomposition shapeConvexDecomposition;
        SoftShape baseSoftShape;

        #endregion

        #region Constructor

        public ConvexDecompositionEngine(SoftCollisionShape softShape)
        {
            baseSoftShape = (SoftShape)((IMapper)softShape).GetShape();
            shapeConvexDecomposition = new ShapeConvexDecomposition(baseSoftShape.AABBox, baseSoftShape.Triangle);
        }

        #endregion

        #region Public Methods

        public List<HashSet<Vertex3Index>> GetConvexShapeList(double precisionSize)
        {
            var vertex = Array.ConvertAll(baseSoftShape.ShapePoints, item => new Vertex3Index(
                                                                                item.Position,
                                                                                item.TriangleIndex.ToArray(), 0));

            return shapeConvexDecomposition.GetConvexShapeList(vertex, precisionSize).Select(x=>x.Vertex3Idx).ToList();
        }

        #endregion
    }
}
