using System;

namespace CollisionEngine
{
	public class CollisionEngineParameters
	{
		#region Collision Engine Parameters Properties
	
		//numero di iterazioni per la ricerca dei punti di collisione
		public int MaxGJKIteration { get; private set; }

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
		}

		public CollisionEngineParameters (
			double collisionDelta,
			int maxGJKIteration,
			int maxEPAIteration,
			double precision,
			double gjkTolerance,
			double epaTolerance,
			int manifoldPointNumber,
			int maxThreadNumber)
		{
			this.MaxGJKIteration = maxGJKIteration;
			this.MaxEPAIteration = maxEPAIteration;
			this.Precision = precision;
			this.GJKManifoldTolerance = gjkTolerance;
			this.EPAManifoldTolerance = epaTolerance;
			this.ManifoldPointNumber = manifoldPointNumber;
			this.MaxThreadNumber = maxThreadNumber;
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

