
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

		#endregion

		#region Constructors

		public SolverParameters ()
		{
			MaxIteration = 20;
			ErrorTolerance = 1E-30;
			SOR = 1.0;
			SORStep = 0.007;
			MaxThreadNumber = 2;
		}

		public SolverParameters (
			int maxIteration,
			double errorTolerance,
			double SOR,
			int maxThreadNumber,
			double SORStep )
		{
			MaxIteration = maxIteration;
			ErrorTolerance = errorTolerance;
			this.SOR = SOR;
			MaxThreadNumber = maxThreadNumber;
			this.SORStep = SORStep;
		}
			
		#endregion

		#region Public Methods

		public void SetSolverMaxIteration(int maxIteration)
		{
			this.MaxIteration = maxIteration;
		}

		public void SetErrorTolerance(double errorTolerance)
		{
			this.ErrorTolerance = errorTolerance;
		}

		public void SetSOR(double successiveOverRelaxation)
		{
			this.SOR = successiveOverRelaxation;
		}

		public void SetSORStep(double successiveOverRelaxationStep)
		{
			this.SOR = successiveOverRelaxationStep;
		}

		#endregion
	}
}

