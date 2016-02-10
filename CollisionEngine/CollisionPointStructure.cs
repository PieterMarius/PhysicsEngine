using System;

namespace ObjectDefinition
{
	public class CollisionPointStructure
	{
		#region Public Properties

		//Indice relativo alla lista di SimulationObject
		public readonly int ObjectA;

		//Indice relativo alla lista di SimulationObject
		public readonly int ObjectB;

		//Distanza tra gli oggetti che stanno collidendo
		public readonly double CollisionDistance;

		//indica se c'è o meno intersezione tra gli oggetti
		public readonly bool Intersection;

		//Distanza di compenetrazione
		public readonly double IntersectionDistance;

		//punto di collisione
		public readonly CollisionPoint CollisionPoint;

		//Lista dei punti di collisione
		public readonly CollisionPoint[] CollisionPoints;

		#endregion

		#region Constructors

		public CollisionPointStructure (
			int objectA,
			int objectB,
			double collisionDistance,
			bool intersection,
			double intersectionDistance,
			CollisionPoint collisionPoint,
			CollisionPoint[] collisionPoints)
		{
			this.ObjectA = objectA;
			this.ObjectB = objectB;
			this.CollisionDistance = collisionDistance;
			this.Intersection = intersection;
			this.IntersectionDistance = intersectionDistance;
			this.CollisionPoint = collisionPoint;
			this.CollisionPoints = collisionPoints;
		}

		#endregion
	}
}

