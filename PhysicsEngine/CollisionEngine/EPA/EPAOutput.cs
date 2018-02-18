
namespace SharpPhysicsEngine.CollisionEngine
{
	internal class EPAOutput
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
			CompenetrationDistance = compenetrationDistance;
			CollisionPoint = collisionPoint;
		}

		#endregion
	}
}

