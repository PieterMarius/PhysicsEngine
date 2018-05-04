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
using System.Diagnostics;

namespace TestPhysics
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			using (TestWindow test = new TestWindow ()) 
			{
                //Test();
                //TestBFS();
				//test.VSync = VSyncMode.Adaptive;
				test.Run(0.0, 0.0);
				
			}
		}

        private static void Test()
        {
            Stopwatch stopwatch = new Stopwatch();

            int nvalue = 38000000;

            Random rnd = new Random();

            SharpEngineMathUtility.Vector3[] testVector1 = new SharpEngineMathUtility.Vector3[nvalue];

            for (int i = 0; i < nvalue; i++)
                testVector1[i] = new SharpEngineMathUtility.Vector3(rnd.NextDouble(), rnd.NextDouble(), rnd.NextDouble());

            stopwatch.Reset();
            stopwatch.Start();

            double test1 = 0.0;
            for (int i = 0; i < nvalue; i++)
                test1 = SharpEngineMathUtility.Vector3.Dot(testVector1[i], testVector1[i]);

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
