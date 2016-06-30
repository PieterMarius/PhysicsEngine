
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

		//parametro di stabilizzazione del motore delle collisioni
		public double Precision { get; private set; }

		//Parametro tolleranza scelta dei punti di collisione
		public double GJKManifoldTolerance { get; private set;}

		//Parametro tolleranza scelta dei punti di collisione
		public double EPAManifoldTolerance { get; private set;}

		//Parametro per stabilire il numero di punti utili per determinare la collisione
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
			MaxGJKIteration = 7;
			MaxEPAIteration = 5;
			Precision = 0.0000001;
			GJKManifoldTolerance = 0.006;
			EPAManifoldTolerance = 0.006;
			ManifoldPointNumber = 4;
			MaxThreadNumber = 4;
			ActivateSweepAndPrune = true;
		}

		public CollisionEngineParameters (
			double collisionDelta,
			int maxGJKIteration,
			int maxEPAIteration,
			double precision,
			double gjkTolerance,
			double epaTolerance,
			int manifoldPointNumber,
			int maxThreadNumber,
			bool activateSweepAndPrune)
		{
			MaxGJKIteration = maxGJKIteration;
			MaxEPAIteration = maxEPAIteration;
			Precision = precision;
			GJKManifoldTolerance = gjkTolerance;
			EPAManifoldTolerance = epaTolerance;
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
			
		#endregion
	}
}

