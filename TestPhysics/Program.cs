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
using SharpEngineMathUtility.Solver;
using System;
using System.Diagnostics;

namespace TestPhysics
{
	class MainClass
	{
		public static void Main (string[] args)
		{
            using (TestWindow test = new TestWindow())
            {
                //Test();
                //TestBFS();
                //test.VSync = VSyncMode.Adaptive;
                //TestGmres();
                test.Run(0.0, 0.0);

            }
		}

        private static void TestGmres()
        {
            var solver = new MINRES();

            SparseElement[] A = new SparseElement[6];
            A[0] = SparseElement.GetSparseElement(new double[] { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            A[1] = SparseElement.GetSparseElement(new double[] { 0.0, 10.0, -3.0, -1.0, 0.0, 0.0 });
            A[2] = SparseElement.GetSparseElement(new double[] { 0.0, 0.0, 15.0, 0.0, 0.0, 0.0 });
            A[3] = SparseElement.GetSparseElement(new double[] { -2.0, 0.0, 0.0, 10.0, -1.0, 0.0 });
            A[4] = SparseElement.GetSparseElement(new double[] { -1.0, 0.0, 0.0, -5.0, 1.0, -3.0 });
            A[5] = SparseElement.GetSparseElement(new double[] { -1.0, -2.0, 0.0, 0.0, 0.0, 6.0 });
                        
            double[] b = new double[] { 10.0, 7.0, 45.0, 33.0, -34.0, 31.0 };
            double[] x = new double[b.Length];

            for (int i = 0; i < x.Length; i++)
            {
                x[i] = 0.0;
            }

            //symmetrize system
            SparseElement[] At = SparseElement.Transpose(A);
            SparseElement[] AA = SparseElement.Square(At);
            double[] Ab = SparseElement.Multiply(At, b);


            var out1 = solver.Solve(AA, Ab, x, 30000);

            var solver1 = new GMRES();

            var out2 = solver1.Solve(A, b, x, 30, 2);


        }

        //private static void TestKMeans()
        //{
        //    SharpPhysicsEngine.K_Means.KMeans engine = new SharpPhysicsEngine.K_Means.KMeans();

        //    //var points = new Vector3[] {
        //    //    new Vector3(1.0,1.0,0.0),
        //    //    new Vector3(1.5,2.0,0.0),
        //    //    new Vector3(3.0,4.0,0.0),
        //    //    new Vector3(5.0,7.0,0.0),
        //    //    new Vector3(3.5,5.0,0.0),
        //    //    new Vector3(4.5,5.0,0.0),
        //    //    new Vector3(3.5,4.5,0.0),
        //    //};

        //    var points = new Vector3[] {
        //        new Vector3(5.0,5.0,0.0),
        //        new Vector3(9,8.0,0.0),
        //        new Vector3(13.0,7.0,0.0),
        //        new Vector3(5.0,12.0,0.0),
        //        new Vector3(10,16.0,0.0),
        //        new Vector3(15,11.0,0.0),
        //        new Vector3(34,22,0.0),
        //        new Vector3(39,21,0.0),
        //        new Vector3(31,27,0.0),
        //        new Vector3(36,26,0.0),
        //        new Vector3(42,27,0.0),
        //        new Vector3(32,30,0.0),
        //        new Vector3(37,30,0.0),
        //        new Vector3(16,30,0.0),
        //        new Vector3(17,28,0.0),
        //        new Vector3(15,31,0.0),
        //        new Vector3(18,32,0.0),
        //        new Vector3(14,25,0.0),
        //    };

        //    var output = engine.Execute(points, 3);

        //}

        private static void Test()
        {
            Stopwatch stopwatch = new Stopwatch();

            int nvalue = 38000000;

            Random rnd = new Random();

            SharpEngineMathUtility.Vector3d[] testVector1 = new SharpEngineMathUtility.Vector3d[nvalue];

            for (int i = 0; i < nvalue; i++)
                testVector1[i] = new SharpEngineMathUtility.Vector3d(rnd.NextDouble(), rnd.NextDouble(), rnd.NextDouble());

            stopwatch.Reset();
            stopwatch.Start();

            double test1 = 0.0;
            for (int i = 0; i < nvalue; i++)
                test1 = SharpEngineMathUtility.Vector3d.Dot(testVector1[i], testVector1[i]);

            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

            //stopwatch.Reset();
            //stopwatch.Start();

            //for (int i = 0; i < nvalue; i++)
            //    test1 = SharpEngineMathUtility.Vector3.SimdDot(testVector1[i], testVector1[i]);

            //stopwatch.Stop();
            //Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

            Console.ReadLine();
        }
    }

    
}
