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
        public static extern bool ExecuteHACD();

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
