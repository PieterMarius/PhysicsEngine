
namespace CollisionEngine
{
	public class CollisionPointStructure
	{
		#region Public Properties

		//Indice relativo alla lista di SimulationObject
		public readonly int ObjectA;

		//Indice relativo alla lista di SimulationObject
		public readonly int ObjectB;

		//Distanza tra gli oggetti che stanno collidendo
		public readonly double ObjectDistance;

		//indica se c'è o meno intersezione tra gli oggetti
		public readonly bool Intersection;

		//punto di collisione
		public readonly CollisionPoint CollisionPoint;

		//Lista dei punti di collisione
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

