using System;
using System.Collections.Generic;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public class SweepAndPrune
	{
		#region Sweep And Prune fields

		private List<EndPoint> vectorX;
		private List<EndPoint> vectorY;
		private List<EndPoint> vectorZ;

		private List<Pair> PairList;

		#endregion

		#region Constructor

		public SweepAndPrune ()
		{
			this.vectorX = new List<EndPoint> ();
			this.vectorY = new List<EndPoint> ();
			this.vectorZ = new List<EndPoint> ();

			this.PairList = new List<Pair> ();
		}

		#endregion

		#region Public Methods

		public List<Pair> GetPairList()
		{
			return null;
		}

		#endregion

		#region Private Methods



		#endregion
	}
}

