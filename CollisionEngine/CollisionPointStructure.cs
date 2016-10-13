
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
		public double ObjectDistance { get; private set; }

		/// <summary>
		/// Check intersection.
		/// </summary>
		public bool Intersection { get; private set; }

		/// <summary>
		/// The collision point.
		/// </summary>
		public CollisionPoint CollisionPoint;

		/// <summary>
		/// The collision points.
		/// </summary>
		public CollisionPoint[] CollisionPoints;

        /// <summary>
        /// Persistent contact time counter
        /// </summary>
        public int FrameCount { get; private set; }

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
            FrameCount = 0;
		}

		#endregion

		#region Public Methods

		public void SetIntersection(bool value)
		{
			Intersection = value;
		}

		public void SetObjectDistance(double value)
		{
			ObjectDistance = value;
		}

        public void SetFrameCount(int count)
        {
            FrameCount = count;
        }

		#endregion

	}
}

