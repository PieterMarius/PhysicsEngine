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

using SharpEngineMathUtility;
using SharpPhysicsEngine.ConvexHullWrapper;
using SharpPhysicsEngine.Terrain;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class TerrainMesh
    {
        #region Fields

        HeightMapMesh terrain;

        #endregion

        #region Constructor

        public TerrainMesh()
        {
            IConvexHullEngine convexHullEngine = new ConvexHullEngine();
            terrain = new HeightMapMesh(
                "heightmap.png", 
                convexHullEngine, 
                0, 
                10, 
                1, 
                50);
        }

        #endregion

        #region Public Methods

        public double[][] GetPositions()
        {
            return MathUtils.GetArrayFromVector3(terrain.GetPosition());
        }

        public double[][] GetNormalArray()
        {
            return MathUtils.GetArrayFromVector3(terrain.GetNormalArray());
        }

        public double[][][] GetTextureCoordMatrix()
        {
            return GeometryUtils.GetMatrixFromVector2Matrix(terrain.GetTextureCoords());
        }

        public double[][][] GetConvexShapeList()
        {
            return MathUtils.GetMatrixFromVector3Matrix(terrain.GetConvexShapes());
        }

        #endregion


    }
}
