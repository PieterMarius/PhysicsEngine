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

using System;
using System.Linq;
using MIConvexHull;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.ConvexHullWrapper
{
    internal sealed class ConvexHullEngine : IConvexHullEngine
    {        
        #region Public Methods

        public ConvexHullData GetConvexHull(Vector3[] points)
        {
            ConvexHullVertex[] vtx = new ConvexHullVertex[points.Length];

            for (int i = 0; i < points.Length; i++)
                vtx[i] = new ConvexHullVertex() { Position = points[i].Array, Index = i };
            
            try
            {
                ConvexHull<ConvexHullVertex, DefaultConvexFace<ConvexHullVertex>> cHull = ConvexHull.Create(vtx);
                var faces = cHull.Faces.ToArray();

                TriangleMesh[] triangleMeshes = new TriangleMesh[faces.Length];
                Vector3[] vertices = Array.ConvertAll(cHull.Points.ToArray(), x=> new Vector3(x.Position));

                foreach (var face in faces.Select((value, i) => new { value, i }))
                {
                    triangleMeshes[face.i] = new TriangleMesh(
                        face.value.Vertices[0].Index,
                        face.value.Vertices[1].Index,
                        face.value.Vertices[2].Index);
                }
                    
                return new ConvexHullData(vertices,triangleMeshes);
            }
            catch (Exception ex)
            {
                throw new Exception(ex.Message, ex);
            }
        }

        #endregion
    }
}