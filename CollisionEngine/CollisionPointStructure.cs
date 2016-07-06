
namespace CollisionEngine
{
	public class CollisionPointStructure
	{
		#region Public Properties

		/// <summary>
		/// The object a.
		/// </summary>
		public readonly int ObjectA;

		/// <summary>
		/// The object b.
		/// </summary>
		public readonly int ObjectB;

		/// <summary>
		/// The object distance.
		/// </summary>
		public readonly double ObjectDistance;

		/// <summary>
		/// Check intersection.
		/// </summary>
		public readonly bool Intersection;

		/// <summary>
		/// The collision point.
		/// </summary>
		public readonly CollisionPoint CollisionPoint;

		/// <summary>
		/// The collision points.
		/// </summary>
		public readonly CollisionPoint[] CollisionPoints;

		#endregion

		#region Constructors

		public CollisionPointStructure (
			int objectA,
			int objectB,
			bool intersection,
			double objectDistance,
			CollisionPoint collisionPoint,
			CollisionPoint[] collisionPoints)
		{
			ObjectA = objectA;
			ObjectB = objectB;
			ObjectDistance = objectDistance;
			Intersection = intersection;
			CollisionPoint = collisionPoint;
			CollisionPoints = collisionPoints;
		}

		#endregion

	}
}

