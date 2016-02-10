using System;
using ObjectDefinition;

namespace CollisionEngine
{
	public class EPAOutput
	{
		#region Public fields

		public readonly double CompenetrationDistance;
		public readonly CollisionPoint CollisionPoint;

		#endregion

		#region Constructor

		public EPAOutput (
			double compenetrationDistance,
			CollisionPoint collisionPoint)
		{
			this.CompenetrationDistance = compenetrationDistance;
			this.CollisionPoint = collisionPoint;
		}

		#endregion
	}
}

