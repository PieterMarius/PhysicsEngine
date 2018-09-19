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
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ExternalFunctionsWrapper
{
    public static class ExtFuncsWrapper
    {
        #region C++ External Functions

        [DllImport("HACD_Wrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool ExecuteHACD(
            string fileName);

        [DllImport("HACD_Wrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool ExtractOFFData(
            string filename,
            [Out] out IntPtr points,
            [Out] out int nPoints,
            [Out] out IntPtr triangles,
            [Out] out int nTriangles,
            bool invert);

        #endregion

        #region Public Methods

        public static void GetOFFData(
            string fileName)
        {
            IntPtr ptrPoints = IntPtr.Zero;
            IntPtr ptrTriangles = IntPtr.Zero;
            bool invert = false;

            bool res = ExtractOFFData(
                "C:\\Users\\vanduin\\Documents\\GitHub\\PhysicsEngine\\TestPhysics\\bin\\x64\\Debug\\Sketched-Brunnen.off", //fileName
                out ptrPoints,
                out int ptrnPoints,
                out ptrTriangles,
                out int ptrnTriangles,
                invert);

            if (res)
            {
                IntPtr[] ptrPointsVec = new IntPtr[ptrnPoints];
                double[][] resultPoint = new double[ptrnPoints][];
                Vector3d[] points = new Vector3d[ptrnPoints];

                Marshal.Copy(
                    ptrPoints,
                    ptrPointsVec,
                    0,
                    ptrnPoints);

                for (int i = 0; i < ptrnPoints; i++)
                {
                    resultPoint[i] = new double[3];

                    Marshal.Copy(
                        ptrPointsVec[i],
                        resultPoint[i],
                        0,
                        3);

                    points[i] = new Vector3d(resultPoint[i]);
                    Marshal.FreeCoTaskMem(ptrPointsVec[i]);
                }

                Marshal.FreeCoTaskMem(ptrPoints);
            }
        }

        #endregion
    }
}
