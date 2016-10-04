
namespace CollisionEngine
{
	public class CollisionEngineParameters
	{
		#region Collision Engine Parameters Properties
	
		/// <summary>
		/// Gets the max GJK iteration.
		/// </summary>
		/// <value>The max GJK iteration.</value>
		public int MaxGJKIteration { get; private set; }

		/// <summary>
		/// Max iteration number for searching the deepest compenetrations point.
		/// </summary>
		/// <value>The max EPA iteration.</value>
		public int MaxEPAIteration { get; private set; }

		/// <summary>
		/// Gets the stabilization parameter for collision engine.
		/// </summary>
		/// <value>The precision.</value>
		public double Precision { get; private set; }

		/// <summary>
		/// Gets the GJK Manifold tolerance.
		/// </summary>
		/// <value>The GJKM anifold tolerance.</value>
		public double GJKManifoldTolerance { get; private set;}

		/// <summary>
		/// Gets the EPA Manifold tolerance parameter.
		/// </summary>
		/// <value>The EPAM anifold tolerance.</value>
		public double EPAManifoldTolerance { get; private set;}

		/// <summary>
		/// Gets the manifold projection tolerance.
		/// </summary>
		/// <value>The manifold projection tolerance.</value>
		public double ManifoldProjectionTolerance { get; private set; }

		/// <summary>
		/// Gets the manifold points number.
		/// </summary>
		/// <value>The manifold point number.</value>
		public int ManifoldPointNumber { get; private set;}

		/// <summary>
		/// Gets the max thread number.
		/// </summary>
		/// <value>The max thread number.</value>
		public int MaxThreadNumber { get; private set;}

		/// <summary>
		/// Gets a value indicating whether this CollisionEngineParameters activate sweep and prune.
		/// </summary>
		/// <value><c>true</c> if activate sweep and prune; otherwise, <c>false</c>.</value>
		public bool ActivateSweepAndPrune { get; private set;}

		#endregion

		#region Constructors

		public CollisionEngineParameters ()
		{
			MaxGJKIteration = 10;
			MaxEPAIteration = 10;
			Precision = 0.0000001;
			GJKManifoldTolerance = 0.009;
			EPAManifoldTolerance = 0.009;
			ManifoldProjectionTolerance = 0.005;
			ManifoldPointNumber = 4;
			MaxThreadNumber = 2;
			ActivateSweepAndPrune = true;
		}

		public CollisionEngineParameters (
			int maxGJKIteration,
			int maxEPAIteration,
			double precision,
			double gjkTolerance,
			double epaTolerance,
			double manifoldProjectionTolerance,
			int manifoldPointNumber,
			int maxThreadNumber,
			bool activateSweepAndPrune)
		{
			MaxGJKIteration = maxGJKIteration;
			MaxEPAIteration = maxEPAIteration;
			Precision = precision;
			GJKManifoldTolerance = gjkTolerance;
			EPAManifoldTolerance = epaTolerance;
			ManifoldProjectionTolerance = manifoldProjectionTolerance;
			ManifoldPointNumber = manifoldPointNumber;
			MaxThreadNumber = maxThreadNumber;
			ActivateSweepAndPrune = activateSweepAndPrune;
		}

		#endregion

		#region Public Methods

		public void SetMaxGJKIteration(int maxIteration)
		{
			MaxGJKIteration = maxIteration;
		}

		public void SetMaxEPAIteration(int maxIteration)
		{
			MaxEPAIteration = maxIteration;
		}
			
		public void SetPrecision(double precision)
		{
			Precision = precision;
		}

		public void SetSweepAndPrune(bool activate)
		{
			ActivateSweepAndPrune = activate;
		}

		public void SetManifoldPoints(int manifoldPointNumber)
		{
			ManifoldPointNumber = manifoldPointNumber;
		}
			
		#endregion
	}
}

