﻿using System;
using PhysicsEngineMathUtility;
using LCPSolver;

namespace MonoPhysicsEngine
{
	public struct Contact
	{
		#region Public Properties

		//Indice relativo alla lista di SimulationObject
		public readonly int ObjectA;

		//Indice relativo alla lista di SimulationObject
		public readonly int ObjectB;

		//Eventuale contatto di riferimento (Es. forza normale nel caso dell'attrito)
		public readonly int ContactReference;

		//Punto di collisione sull'oggetto A
		public readonly Vector3 CollisionPoint;

		//Normale della collisione dell'oggetto B
		public readonly Vector3 Normal;

		/// <summary>
		/// The type of contact (collision, friction, joint).
		/// </summary>
		public readonly ConstraintType Type;

		/// <summary>
		/// The expeceted known value.
		/// </summary>
		public readonly double B;

		/// <summary>
		/// The constraint limit.
		/// </summary>
		public readonly double ConstraintLimit;

		/// <summary>
		/// The suggested start value for solver solution find.
		/// </summary>
		public readonly double Solution;

		#endregion

		#region Constructor

		public Contact (
			int objectA,
			int objectB,
			int contactReference,
			Vector3 collisionPoint,
			Vector3 normal,
			ConstraintType type,
			double b,
			double constraintLimit,
			double solution)
		{
			this.ObjectA = objectA;
			this.ObjectB = objectB;
			this.ContactReference = contactReference;
			this.CollisionPoint = collisionPoint;
			this.Normal = normal;
			this.Type = type;
			this.B = b;
			this.ConstraintLimit = constraintLimit;
			this.Solution = solution;
		}

		#endregion
	}
}

