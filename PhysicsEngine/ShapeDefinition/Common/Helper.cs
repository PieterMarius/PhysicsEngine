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
using System;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal static class Helper
    {
        public static Vector3 GetVertexPosition(
            IShape obj,
            int vertexIndex)
        {
            return
                obj.Position +
                (obj.RotationMatrix * obj.VerticesRelPos[vertexIndex]);
        }

        public static Vector3 GetVertexPosition(
            IShape obj,
            Vector3 relativePos)
        {
            return
                obj.Position +
                (obj.RotationMatrix * relativePos);
        }


        public static IGeometry[] GetGeometry(IShape shape)
        {
            if (shape is IConvexShape convexShape)
                return new IGeometry[] { convexShape.ObjectGeometry };

            if (shape is ICompoundShape compoundShape)
                return compoundShape.ShapesGeometry;
                        
            if (shape is IConcaveShape concaveShape)
                return concaveShape.ConvexShapesGeometry;

            throw new ArgumentException("Unexpected type: " + shape.GetType());
        }
    }
}
