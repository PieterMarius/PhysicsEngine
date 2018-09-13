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
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Globalization;

namespace Utility
{
    public static class GenericUtility
    {
        public class ObjProperties
        {
            public Vector3d[] vertexPoint;
            public int[][] triangleIndex;
        }

        public static double InvariantCultureDoubleConverter(string value)
        {
            string text = value.Replace(',', '.');
            double.TryParse(text, NumberStyles.Any, CultureInfo.InvariantCulture, out double tmp);
            return tmp;
        }
                
        public static ObjProperties GetImportedObjectProperties(
            string fileName,
            double scale)
        {
            ObjImporter importer = new ObjImporter();
            ObjImporter.meshStruct mesh = importer.ImportFile(fileName);

            Vector3d[] vertexPoint = new Vector3d[mesh.vertices.Length];

            for (int i = 0; i < mesh.vertices.Length; i++)
                vertexPoint[i] = mesh.vertices[i];

            OpenGLUtilities.UnitizeObject(ref vertexPoint);
            OpenGLUtilities.ScaleObject(ref vertexPoint, scale);

            int nTriangle = mesh.faceData.Length / 3;
            int[][] triangleIndex = new int[nTriangle][];

            for (int i = 0; i < nTriangle; i++)
            {
                triangleIndex[i] = new int[] {
                    (int)mesh.faceData[i * 3].x - 1,
                    (int)mesh.faceData[(i * 3) + 1].x - 1,
                    (int)mesh.faceData[(i * 3) + 2].x - 1 };
            }
            
            return new ObjProperties { vertexPoint = vertexPoint, triangleIndex = triangleIndex };
        }
    }
}
