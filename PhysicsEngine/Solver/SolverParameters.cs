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

namespace SharpPhysicsEngine.LCPSolver
{
	public class SolverParameters
	{
		#region Solver Parameters Properties

		/// <summary>
		/// Gets the max iteration.
		/// </summary>
		/// <value>The max iteration.</value>
		public int MaxIteration { get; private set; }

		/// <summary>
		/// Gets the error tolerance.
		/// </summary>
		/// <value>The error tolerance.</value>
		public double ErrorTolerance { get; private set; }

		/// <summary>
		/// Gets Successive Over Relaxation term
		/// </summary>
		/// <value>The SO.</value>
		public double SOR { get; private set; }
        
        /// <summary>
		/// Gets the max thread number.
		/// </summary>
		/// <value>The max thread number.</value>
		public int MaxThreadNumber { get; private set; }
        
        #endregion

        #region Constructors

        public SolverParameters ()
		{
			MaxIteration = 30;
			ErrorTolerance = 1E-5;
			SOR = 1.0;
			MaxThreadNumber = 8;
        }

		public SolverParameters (
			int maxIteration,
			double errorTolerance,
			double sor,
			int maxThreadNumber)
		{
			MaxIteration = maxIteration;
			ErrorTolerance = errorTolerance;
			SOR = sor;
			MaxThreadNumber = maxThreadNumber;
        }
			
		#endregion

		#region Public Methods

		public void SetSolverMaxIteration(int maxIteration)
		{
			MaxIteration = maxIteration;
		}

		public void SetErrorTolerance(double errorTolerance)
		{
			ErrorTolerance = errorTolerance;
		}

		public void SetSOR(double successiveOverRelaxation)
		{
			SOR = successiveOverRelaxation;
		}
        		
		#endregion
	}
}

