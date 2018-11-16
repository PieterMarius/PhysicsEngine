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
using SharpPhysicsEngine.LCPSolver;
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
                //MultiplyMat();
                //TestGmres();
                test.Run(0.0, 0.0);

            }
		}

        private static void MultiplyMat()
        {
            SparseMatrix A = new SparseMatrix(2, 3);
            A.Rows[0] = SparseVector.GetSparseElement(new double[] { 1, 2, 3 });
            A.Rows[1] = SparseVector.GetSparseElement(new double[] { 4, 5, 6 });

            SparseMatrix B = new SparseMatrix(3, 2);
            B.Rows[0] = SparseVector.GetSparseElement(new double[] { 7, 8 });
            B.Rows[1] = SparseVector.GetSparseElement(new double[] { 9, 10 });
            B.Rows[2] = SparseVector.GetSparseElement(new double[] { 11, 12 });

            var res = SparseMatrix.Multiply(A, B);
        }

        private static void TestGmres()
        {
            var luSolver = new LUSolver();

            var solver = new MINRES();
            var cg = new ConjugateGradient();

            SparseMatrix A = new SparseMatrix(6, 6);
            A.Rows[0] = SparseVector.GetSparseElement(new double[] { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            A.Rows[1] = SparseVector.GetSparseElement(new double[] { 0.0, 10.0, -3.0, -1.0, 0.0, 0.0 });
            A.Rows[2] = SparseVector.GetSparseElement(new double[] { 0.0, 0.0, 15.0, 0.0, 0.0, 0.0 });
            A.Rows[3] = SparseVector.GetSparseElement(new double[] { -2.0, 0.0, 0.0, 10.0, -1.0, 0.0 });
            A.Rows[4] = SparseVector.GetSparseElement(new double[] { -1.0, 0.0, 0.0, -5.0, 1.0, -3.0 });
            A.Rows[5] = SparseVector.GetSparseElement(new double[] { -1.0, -2.0, 0.0, 0.0, 0.0, 6.0 });
                        
            double[] b = new double[] { 10.0, 7.0, 45.0, 33.0, -34.0, 31.0 };
            double[] x = new double[b.Length];

            for (int i = 0; i < x.Length; i++)
            {
                x[i] = 0.0;
            }

            //symmetrize system
            SparseMatrix At = SparseMatrix.Transpose(A);
            //SparseMatrix AA = SparseMatrix.Square(At);
            SparseMatrix AA = SparseMatrix.Multiply(At, A);
            double[] Ab = SparseMatrix.Multiply(At, b);

            var cgout = solver.Solve(AA, Ab, x, 10);
            var out1 = solver.Solve(AA, Ab, x, 30000);

            var solver1 = new GMRES();

            var out2 = solver1.Solve(A, b, x, 30, 6);
            var lout = luSolver.Solve(A, b, out bool valid);

            HouseholderQR hs = new HouseholderQR();

            SparseMatrix B = new SparseMatrix(5, 3);
            B.Rows[0] = SparseVector.GetSparseElement(new double[] { 12.0, -51.0, 4.0 });
            B.Rows[1] = SparseVector.GetSparseElement(new double[] { 6.0, 167.0, -68.0 });
            B.Rows[2] = SparseVector.GetSparseElement(new double[] { -4.0, 24.0, -41.0 });
            B.Rows[3] = SparseVector.GetSparseElement(new double[] { -1.0, 1.0, 0.0 });
            B.Rows[4] = SparseVector.GetSparseElement(new double[] { 2.0, 0.0, 3.0 });

            hs.Householder(B);
            hs.Solve(A, b);

            Lemke lm = new Lemke(SharpEngineMathUtility.Solver.SolverType.HouseHolderQR);

            SparseMatrix M = new SparseMatrix(3, 3);
            M.Rows[0] = SparseVector.GetSparseElement(new double[] { 21.0, 0.0, 0.0 });
            M.Rows[1] = SparseVector.GetSparseElement(new double[] { 28.0, 14.0, 0.0 });
            M.Rows[2] = SparseVector.GetSparseElement(new double[] { 24.0, 24.0, 12.0 });

            double[] q = new double[] { -1.0, -1.0, -1.0 };

            lm.Solve(M, q, 10);

            SparseMatrix M1 = new SparseMatrix(2, 2);
            M1.Rows[0] = SparseVector.GetSparseElement(new double[] { 2.0, 1.0 });
            M1.Rows[1] = SparseVector.GetSparseElement(new double[] { 1.0, 2.0 });

            double[] q1 = new double[] { -5.0, -6.0 };

            lm.Solve(M1, q1, 10);

            double[] q2 = new double[] { 1.0, 2.0 };

            lm.Solve(M1, q2, 10);

            SparseMatrix M3 = new SparseMatrix(3, 3);
            M3.Rows[0] = SparseVector.GetSparseElement(new double[] { 1.0, 0.0, 2.0 });
            M3.Rows[1] = SparseVector.GetSparseElement(new double[] { 3.0, 2.0, -1.0 });
            M3.Rows[2] = SparseVector.GetSparseElement(new double[] { -2.0, 1.0, 0.0 });

            double[] q3 = new double[] { - 1.0, 2.0, -3.0 };

            lm.Solve(M3, q3, 10);

            SparseMatrix M4 = new SparseMatrix(3, 3);
            M4.Rows[0] = SparseVector.GetSparseElement(new double[] { 0.0, 0.0, 1.0 });
            M4.Rows[1] = SparseVector.GetSparseElement(new double[] { 0.0, 2.0, 1.0 });
            M4.Rows[2] = SparseVector.GetSparseElement(new double[] { -1.0, -1.0, 0.0 });

            double[] q4 = new double[] { -6.0, 0.0, 4.0 };

            lm.Solve(M4, q4, 10);

            SparseMatrix M5 = new SparseMatrix(3, 3);
            M5.Rows[0] = SparseVector.GetSparseElement(new double[] { 1.0, 2.0, 0.0 });
            M5.Rows[1] = SparseVector.GetSparseElement(new double[] { 0.0, 1.0, 2.0 });
            M5.Rows[2] = SparseVector.GetSparseElement(new double[] { 2.0, 0.0, 1.0 });

            double[] q5 = new double[] { -1.0, -1.0, -1.0 };

            lm.Solve(M5, q5, 10);

            SparseMatrix M6 = new SparseMatrix(4, 4);
            M6.Rows[0] = SparseVector.GetSparseElement(new double[] { 1.0, 1.0, 3.0, 4.0 });
            M6.Rows[1] = SparseVector.GetSparseElement(new double[] { 5.0, 3.0, 1.0, 1.0 });
            M6.Rows[2] = SparseVector.GetSparseElement(new double[] { 2.0, 1.0, 2.0, 2.0 });
            M6.Rows[3] = SparseVector.GetSparseElement(new double[] { 1.0, 4.0, 1.0, 1.0 });

            double[] q6 = new double[] { -1.0, 2.0, 1.0, 3.0 };

            lm.Solve(M6, q6, 10);
                        
            SparseMatrix M8 = new SparseMatrix(6, 6);
            M8.Rows[0] = SparseVector.GetSparseElement(new double[] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 });
            M8.Rows[1] = SparseVector.GetSparseElement(new double[] { 2.0, 4.0, 8.0, 16.0, 32.0, 64.0 });
            M8.Rows[2] = SparseVector.GetSparseElement(new double[] { 3.0, 9.0, 27.0, 81.0, 243.0, 729.0 });
            M8.Rows[3] = SparseVector.GetSparseElement(new double[] { 4.0, 16.0, 64.0, 256.0, 1024.0, 4096.0 });
            M8.Rows[4] = SparseVector.GetSparseElement(new double[] { 5.0, 25.0, 125.0, 625.0, 3125.0, 15625.0 });
            M8.Rows[5] = SparseVector.GetSparseElement(new double[] { 6.0, 36.0, 216.0, 1296.0, 7776.0, 46656.0 });

            var ll1 = luSolver.Solve(M8, new double[6], out bool valid1);
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
