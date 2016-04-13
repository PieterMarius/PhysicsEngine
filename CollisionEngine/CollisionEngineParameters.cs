using System;

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
		/// Gets a value indicating whether this <see cref="CollisionEngine.CollisionEngineParameters"/> activate sweep and prune.
		/// </summary>
		/// <value><c>true</c> if activate sweep and prune; otherwise, <c>false</c>.</value>
		public bool ActivateSweepAndPrune { get; private set;}

		#endregion

		#region Constructors

		public CollisionEngineParameters ()
		{
			this.MaxGJKIteration = 7;
			this.MaxEPAIteration = 50;
			this.Precision = 0.0000001;
			this.GJKManifoldTolerance = 0.006;
			this.EPAManifoldTolerance = 0.006;
			this.ManifoldPointNumber = 4;
			this.MaxThreadNumber = 2;
			this.ActivateSweepAndPrune = true;
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
			this.MaxGJKIteration = maxGJKIteration;
			this.MaxEPAIteration = maxEPAIteration;
			this.Precision = precision;
			this.GJKManifoldTolerance = gjkTolerance;
			this.EPAManifoldTolerance = epaTolerance;
			this.ManifoldPointNumber = manifoldPointNumber;
			this.MaxThreadNumber = maxThreadNumber;
			this.ActivateSweepAndPrune = activateSweepAndPrune;
		}

		#endregion

		#region Public Methods

		public void SetMaxGJKIteration(int maxIteration)
		{
			this.MaxGJKIteration = maxIteration;
		}

		public void SetMaxEPAIteration(int maxIteration)
		{
			this.MaxEPAIteration = maxIteration;
		}
			
		public void SetPrecision(double precision)
		{
			this.Precision = precision;
		}
			
		#endregion
	}
}

