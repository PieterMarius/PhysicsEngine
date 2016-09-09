﻿
namespace LCPSolver
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
		/// Gets the Successive Over Relaxation decrease step.
		/// </summary>
		/// <value>The SOR step.</value>
		public double SORStep { get; private set; }

		/// <summary>
		/// Gets the max thread number.
		/// </summary>
		/// <value>The max thread number.</value>
		public int MaxThreadNumber { get; private set; }

		/// <summary>
		/// Gets a value indicating whether this LCPSolver early exit.
		/// </summary>
		/// <value><c>true</c> if early exit; otherwise, <c>false</c>.</value>
		public bool EarlyExit { get; private set; }

		#endregion

		#region Constructors

		public SolverParameters ()
		{
			MaxIteration = 10;
			ErrorTolerance = 1E-15;
			SOR = 1.0;
			SORStep = 0.007;
			MaxThreadNumber = 2;
			EarlyExit = false;
		}

		public SolverParameters (
			int maxIteration,
			double errorTolerance,
			double sor,
			int maxThreadNumber,
			double sorStep,
			bool earlyExit)
		{
			MaxIteration = maxIteration;
			ErrorTolerance = errorTolerance;
			SOR = sor;
			MaxThreadNumber = maxThreadNumber;
			SORStep = sorStep;
			EarlyExit = earlyExit;
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

		public void SetSORStep(double successiveOverRelaxationStep)
		{
			SORStep = successiveOverRelaxationStep;
		}

		#endregion
	}
}

