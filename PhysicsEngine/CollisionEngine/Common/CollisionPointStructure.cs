
namespace SharpPhysicsEngine.CollisionEngine
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
        /// Collision Points Base Structure
        /// </summary>
        public CollisionPointBaseStructure[] CollisionPointBase { get; private set; }

        /// <summary>
        /// Persistent contact time counter
        /// </summary>
        public int FrameCount { get; private set; }

		#endregion

		#region Constructors

		public CollisionPointStructure (
			int objectA,
			int objectB,
            CollisionPointBaseStructure[] collisionPointBase)
		{
			ObjectA = objectA;
			ObjectB = objectB;
            CollisionPointBase = collisionPointBase;
            FrameCount = 0;
		}

        public CollisionPointStructure(
            int objectA,
            int objectB,
            CollisionPointBaseStructure collisionPointBase)
            : this(objectA, objectB, new CollisionPointBaseStructure[] { collisionPointBase })
        { }

        #endregion

        #region Public Methods

        public void SetFrameCount(int count)
        {
            FrameCount = count;
        }

        public void SetBaseCollisionPoint(
            CollisionPointBaseStructure[] collisionPointBase)
        {
            CollisionPointBase = collisionPointBase;
        }

		#endregion

	}

    public class CollisionPointBaseStructure
    {
        #region Fields

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

        #endregion

        #region Constructor

        public CollisionPointBaseStructure(
            double objectDistance,
            bool intersection,
            CollisionPoint collisionPoint,
            CollisionPoint[] collisionPoints)
        {
            ObjectDistance = objectDistance;
            Intersection = intersection;
            CollisionPoint = collisionPoint;
            CollisionPoints = collisionPoints;
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

        #endregion

    }
}

