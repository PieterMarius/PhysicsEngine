using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace CollisionEngine
{
	public class GJK
	{

		#region Settings Variable

		public int MaxIterations { get; private set; }
		public double Precision { get; private set; }
		public double GJKManifoldTolerance { get; private set;}
		public int ManifoldPointNumber { get; private set;}

		static readonly Vector3 origin = new Vector3();
		static readonly double constTolerance = 0.0000001;

		#endregion

		#region Constructors

		public GJK (
			int maxIterations,
			double precision,
			double gjkTolerance,
			int manifoldPointNumber)
		{
			MaxIterations = maxIterations;
			Precision = precision;
			GJKManifoldTolerance = gjkTolerance;
			ManifoldPointNumber = manifoldPointNumber;
		}

		#endregion

		#region "Private Methods"

		/// <summary>
		/// Gets the farthest point into solid, specified by direction
		/// </summary>
		/// <returns>The farthest point.</returns>
		/// <param name="obj">Object.</param>
		/// <param name="direction">Direction.</param>
		private int GetFarthestPoint(
			ObjectGeometry obj, 
			Vector3 direction)
		{
			int index = 0;
			double maxDot = Vector3.Dot (obj.VertexPosition [index], direction);
			double dot = 0.0;

			for (int i = 1; i < obj.VertexPosition.Length; i++) 
			{
				dot = Vector3.Dot (obj.VertexPosition [i], direction);

				if (dot > maxDot) 
				{
					maxDot = dot;
					index = i;
				}
			}
			return index;
		}
			
		/// <summary>
		/// Gets the minkowski farthest point.
		/// </summary>
		/// <returns>The minkowski farthest point.</returns>
		/// <param name="obj1">Obj1.</param>
		/// <param name="obj2">Obj2.</param>
		/// <param name="direction">Direction.</param>
		private Support GetMinkowskiFarthestPoint(
			ObjectGeometry obj1, 
			ObjectGeometry obj2,
			Vector3? direction)
		{
			
			int a = GetFarthestPoint (obj1, direction.Value);
			int b = GetFarthestPoint (obj2, direction.Value * -1.0);

			var sp = new Support (
				             obj1.VertexPosition [a] - obj2.VertexPosition [b],
				             a,
				             b);

			return sp;
		}

		private Vector3 GetDirectionOnSimplex2(Simplex simplex)
		{
			Vector3 simplexAB = simplex.Support [1].s - simplex.Support [0].s;
			Vector3 simplexAO = simplex.Support [0].s * - 1.0;

			if (Vector3.Dot(simplexAB,simplexAO) > 0.0)
				return Vector3.Cross(
					Vector3.Cross(simplexAB, simplexAO), 
					simplexAB);
			
			return simplexAO;
		}

		private Vector3? GetDirectionOnSimplex3(ref Simplex simplex)
		{
			Vector3 simplexAB = simplex.Support [1].s - simplex.Support [0].s;
			Vector3 simplexAC = simplex.Support [2].s - simplex.Support [0].s; 
			Vector3 simplexAO = origin - simplex.Support [0].s;
			Vector3 ABC = Vector3.Cross (simplexAB, simplexAC);

			if (Vector3.Dot (simplexAO, Vector3.Cross (simplexAB, ABC)) > 0.0) 
			{
				return Vector3.Cross (
					Vector3.Cross (simplexAB, simplexAO), 
					simplexAB);
			}

			if (Vector3.Dot (simplexAO, Vector3.Cross (ABC, simplexAC)) > 0.0) 
			{
				simplex.Support [1] = simplex.Support [2];
				return Vector3.Cross (
					Vector3.Cross (simplexAC, simplexAO), 
					simplexAC);
			}

			return null;
		}
			
		private bool EvaluateOrigionOnSimplex4(Simplex simplex)
		{
			Vector3[] vt = new Vector3[4];
			vt[0] = new Vector3(simplex.Support[0].s);
			vt[1] = new Vector3(simplex.Support[1].s);
			vt[2] = new Vector3(simplex.Support[2].s);
			vt[3] = new Vector3(simplex.Support[3].s);

			double[] t = new double[24];

			t[0] = vt[2].x * vt[1].y * vt[0].z;
			t[1] = vt[3].x * vt[1].y * vt[0].z;
			t[2] = vt[1].x * vt[2].y * vt[0].z;
			t[3] = vt[3].x * vt[2].y * vt[0].z;
			t[4] = vt[1].x * vt[3].y * vt[0].z;
			t[5] = vt[2].x * vt[3].y * vt[0].z;

			t[6] = vt[2].x * vt[0].y * vt[1].z;
			t[7] = vt[3].x * vt[0].y * vt[1].z;
			t[8] = vt[0].x * vt[2].y * vt[1].z;
			t[9] = vt[3].x * vt[2].y * vt[1].z;
			t[10] = vt[0].x * vt[3].y * vt[1].z;
			t[11] = vt[2].x * vt[3].y * vt[1].z;

			t[12] = vt[1].x * vt[0].y * vt[2].z;
			t[13] = vt[3].x * vt[0].y * vt[2].z;
			t[14] = vt[0].x * vt[1].y * vt[2].z;
			t[15] = vt[3].x * vt[1].y * vt[2].z;
			t[16] = vt[0].x * vt[3].y * vt[2].z;
			t[17] = vt[1].x * vt[3].y * vt[2].z;

			t[18] = vt[1].x * vt[0].y * vt[3].z;
			t[19] = vt[2].x * vt[0].y * vt[3].z;
			t[20] = vt[0].x * vt[1].y * vt[3].z;
			t[21] = vt[2].x * vt[1].y * vt[3].z;
			t[22] = vt[0].x * vt[2].y * vt[3].z;
			t[23] = vt[1].x * vt[2].y * vt[3].z;

			double d0 = -t[0] + t[1] + t[2] - t[3] - t[4] + t[5] +
				t[6] - t[7] - t[8] + t[9] + t[10] - t[11] -
				t[12] + t[13] + t[14] - t[15] - t[16] + t[17]+
				t[18] - t[19] - t[20] + t[21] + t[22] - t[23];

			//Test if points are aligned or not
			if (Math.Abs (d0) < constTolerance) 
			{
				return false;
			}
			
			double d1 = t[9] - t[11] - t[15] + t[17] + t[21] - t[23];

			double d2 = -t[3] + t[5] + t[13] - t[16] - t[19] + t[22];

			double d3 = t[1] - t[4] - t[7] + t[10] + t[18] - t[20];

			double d4 = -t[0] + t[2] + t[6] - t[8] - t[12] + t[14];

			if (GeometryUtilities.Sign (d0, d1) &&
			   GeometryUtilities.Sign (d1, d2) &&
			   GeometryUtilities.Sign (d2, d3) &&
			   GeometryUtilities.Sign (d3, d4)) 
				return true;
			
			return false;		
		}

		private CollisionPoint GetCoordinatesFromMinkowsky(
			Simplex simplex,
			ObjectGeometry shape1,
			ObjectGeometry shape2,
			Vector3 normal)
		{
			
			Vector3 ba1 = shape1.VertexPosition[simplex.Support[1].a] - shape1.VertexPosition[simplex.Support[0].a];
			Vector3 ca1 = shape1.VertexPosition[simplex.Support[2].a] - shape1.VertexPosition[simplex.Support[0].a];

			Vector3 ba2 = shape2.VertexPosition[simplex.Support[1].b] - shape2.VertexPosition[simplex.Support[0].b];
			Vector3 ca2 = shape2.VertexPosition[simplex.Support[2].b] - shape2.VertexPosition[simplex.Support[0].b];

			return new CollisionPoint (
				shape1.VertexPosition [simplex.Support [0].a] + (ba1 * simplex.w) + (ca1 * simplex.t),
				shape2.VertexPosition [simplex.Support [0].b] + (ba2 * simplex.w) + (ca2 * simplex.t),
				normal);
		}
			
		/// <summary>
		/// Gets the distance on simplex4.
		/// </summary>
		/// <returns>The distance on simplex4.</returns>
		/// <param name="startDistance">Start distance.</param>
		/// <param name="simplex">Simplex.</param>
		private Vector3? GetDistanceOnSimplex4(
			double startDistance,
			ref Simplex simplex)
		{
			Simplex[] t = new Simplex[4];

			t [0] = new Simplex ();
			t [0].Support [0] = simplex.Support [0];
			t [0].Support [1] = simplex.Support [1];
			t [0].Support [2] = simplex.Support [3];

			t [1] = new Simplex ();
			t [1].Support [0] = simplex.Support [0];
			t [1].Support [1] = simplex.Support [2];
			t [1].Support [2] = simplex.Support [3];

			t [2] = new Simplex ();
			t [2].Support [0] = simplex.Support [1];
			t [2].Support [1] = simplex.Support [2];
			t [2].Support [2] = simplex.Support [3];

			t [3] = new Simplex ();
			t [3].Support [0] = simplex.Support [0];
			t [3].Support [1] = simplex.Support [1];
			t [3].Support [2] = simplex.Support [2];

			Vector3? distanceVector = new Vector3 ();
			Vector3? mDistance = null;
			double minDistance = startDistance;
			double distance = 0.0;

			double st = 0.0;
			double tt = 0.0;
		
			for (int i = 0; i < 4; i++) {

				if (t [i].Support [0].s == t [i].Support [1].s ||
					t [i].Support [0].s == t [i].Support [2].s ||
					t [i].Support [1].s == t [i].Support [2].s)
					continue;

				// Test if points are aligned
				if (!GeometryUtilities.TestCollinearity (
					    t [i].Support [0].s,
					    t [i].Support [1].s,
					    t [i].Support [2].s)) {

					distanceVector = GeometryUtilities.GetPointTriangleIntersection (
						t [i].Support [0].s,
						t [i].Support [1].s,
						t [i].Support [2].s,
						origin,
						ref st,
						ref tt);

					if (!distanceVector.HasValue)
						continue;

					t [i].w = st;
					t [i].t = tt;

					distance = Vector3.Length (distanceVector.Value);

					if (distance <= minDistance) {
						mDistance = distanceVector.Value;
						minDistance = distance;
						simplex = (Simplex)t [i].Clone ();
					}
				}
			}

			return mDistance;
		}

		private Simplex FindAndTestSimplex4(
			ObjectGeometry shape1,
			ObjectGeometry shape2,
			Simplex simplex,
			Vector3 direction,
			ref bool isIntersection)
		{
			//Aggiungo il quarto punto al tetraedro 
			simplex.Support[3] = GetMinkowskiFarthestPoint (shape1, shape2, direction);

			isIntersection |= EvaluateOrigionOnSimplex4(simplex);

			return simplex;
		}

		/// <summary>
		/// Ritorna la distanza se distanza 0.0f allora vi è intersezione o compenetrazione tra gli oggetti, 
		/// se distanza 0.0 allora i due oggetti non collidono
		/// </summary>
		/// <returns>The GJK algorithm.</returns>
		/// <param name="shape1">Shape1.</param>
		/// <param name="shape2">Shape2.</param>
		/// <param name="cp">Cp.</param>
		/// <param name="minSimplex">Minimum simplex.</param>
		/// <param name="isIntersection">If set to <c>true</c> is itersection.</param>
		private double ExecuteGJKAlgorithm(
			ObjectGeometry shape1,
			ObjectGeometry shape2,
			ref Vector3 collisionNormal,
			ref CollisionPoint cp,
			ref Simplex minSimplex,
			ref bool isIntersection)
		{
			double minDistance = double.MaxValue;
			double s = 0.0, t = 0.0;

			var simplex = new Simplex ();

			Vector3? direction = GeometryUtilities.GetRandomDirection ();

			//Primo punto del simplex

			simplex.Support [0] = GetMinkowskiFarthestPoint (shape1, shape2, direction);

			//Secondo punto del simplex

			direction = direction * -1.0;

			simplex.Support[1] = GetMinkowskiFarthestPoint (shape1, shape2, direction);

			//Terzo punto del simplex

			direction = Vector3.Normalize (GetDirectionOnSimplex2 (simplex));

			simplex.Support[2] = GetMinkowskiFarthestPoint (shape1, shape2, direction);

			double mod = minDistance;

			for (int i = 0; i < MaxIterations; i++) 
			{
				//Verifico la direzione e se aggiungere o meno il quarto punto
				direction = GetDirectionOnSimplex3 (ref simplex);

				if (direction.HasValue) 
				{
					direction = Vector3.Normalize (GetDirectionOnSimplex2 (simplex));

					simplex.Support[2] = GetMinkowskiFarthestPoint (shape1, shape2, direction);

					if (GeometryUtilities.TestCollinearity (
						simplex.Support [0].s,
						simplex.Support [1].s,
						simplex.Support [2].s))
					{
						#region patological case

						while (GeometryUtilities.TestCollinearity (
							simplex.Support [0].s,
							simplex.Support [1].s,
							simplex.Support [2].s)) 
						{
							direction = GeometryUtilities.GetRandomDirection ();

							//Modifico il simplex
							simplex.Support[2] = GetMinkowskiFarthestPoint (shape1, shape2, direction);
						}

						#endregion
					}
					else
					{
						continue;
					}
				}
					
				direction = GeometryUtilities.GetPointTriangleIntersection (
					simplex.Support [0].s,
					simplex.Support [1].s,
					simplex.Support [2].s,
					origin,
					ref s,
					ref t);

				if (!direction.HasValue) 
				{
					direction = GeometryUtilities.GetRandomDirection ();
				}

				direction = -1.0 * direction;

				//Check validity of direction
				if (Vector3.Length (direction.Value) < constTolerance)
				{
					#region patological case

					//Calcolo la normale del triangolo
					Vector3 triangleNormal = GeometryUtilities.CalculateNormal (
						                    simplex.Support [0].s,
						                    simplex.Support [1].s,
						                    simplex.Support [2].s);

					simplex = FindAndTestSimplex4 (
						shape1,
						shape2,
						simplex,
						triangleNormal,
						ref isIntersection);

					if (isIntersection)
					{
						minSimplex = (Simplex) simplex.Clone ();
						return -1.0;
					}

					simplex = FindAndTestSimplex4 (
						shape1,
						shape2,
						simplex,
						triangleNormal * -1.0,
						ref isIntersection);

					if (isIntersection)
					{
						minSimplex = (Simplex) simplex.Clone ();
						return -1.0;
					}
						
					#endregion
				}

				direction = Vector3.Normalize (direction.Value);

				simplex = FindAndTestSimplex4 (
					shape1,
					shape2,
					simplex,
					direction.Value,
					ref isIntersection);

				//Verifico che contenga l'origine
				if (isIntersection) 
				{
					//L'origine è contenuta nel poliedro
					minSimplex = (Simplex) simplex.Clone ();
					return -1.0;
				}
					
				Vector3? p = GetDistanceOnSimplex4 (mod, ref simplex);

				if (!p.HasValue) 
				{
					direction = GeometryUtilities.GetRandomDirection ();
					simplex.Support[3] = GetMinkowskiFarthestPoint (shape1, shape2, direction);
					continue;
				}

				mod = Vector3.Length (p.Value);

				//cambio direzione di ricerca
				direction = Vector3.Normalize (-1.0 * p.Value);

				//early exit
				if (simplex.Equals (minSimplex))
					break;

				// Prendo il punto più vicino
				if (mod < minDistance) 
				{
					collisionNormal = p.Value * - 1.0;
					minDistance = mod;
					minSimplex = (Simplex) simplex.Clone ();
				}
			}

			collisionNormal = collisionNormal.Normalize ();

			cp = GetCoordinatesFromMinkowsky (
				minSimplex, 
				shape1, 
				shape2, 
				collisionNormal);
			
			return minDistance;
		}
			
		#endregion

		#region Public Methods

		/// <summary>
		/// Detects the collision.
		/// </summary>
		/// <returns>The collision.</returns>
		/// <param name="objectA">Object a.</param>
		/// <param name="objectB">Object b.</param>
		public GJKOutput Execute(
			ObjectGeometry objectA, 
			ObjectGeometry objectB)
		{
			var collisionPoint = new CollisionPoint();
			var collisionNormal = new Vector3();
			var minSimplex = new Simplex();
			bool isIntersection = false;

			double collisionDistance = ExecuteGJKAlgorithm (
				                          objectA,
				                          objectB,
				                          ref collisionNormal,
				                          ref collisionPoint,
				                          ref minSimplex,
				                          ref isIntersection);

			return new GJKOutput (
				collisionDistance,
				collisionPoint,
				collisionNormal,
				isIntersection,
				minSimplex);
		}
			

		#endregion

	}
}

