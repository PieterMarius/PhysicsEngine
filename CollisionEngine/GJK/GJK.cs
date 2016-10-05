using System.Collections.Generic;
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

		readonly Vector3 origin = new Vector3();
		readonly double constTolerance = 0.0000001;
        
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
		/// Gets the farthest vertex point between two input objects.
		/// </summary>
		/// <returns>The farthest point.</returns>
		/// <param name="objA">Object a.</param>
		/// <param name="objB">Object b.</param>
		private Support GetFarthestPoint(
			SimulationObject objA,
			SimulationObject objB)
		{
			int indexA = 0;
			int indexB = objB.RelativePositions.Length / 2;
			
			return new Support(
				Helper.GetVertexPosition(objA,indexA) - Helper.GetVertexPosition(objB, indexB),
				indexA,
				indexB);
		}

		private Vector3 GetDirectionOnSimplex2(Simplex simplex)
		{
			Vector3 simplexAB = simplex.Support [1].s - simplex.Support [0].s;
			Vector3 simplexAO = simplex.Support [0].s * - 1.0;

			return Vector3.Cross(
					Vector3.Cross(simplexAB, simplexAO), 
					simplexAB);
		}

		private Vector3 GetMinDistance(
			ref List<SupportTriangle> triangles,
			Vector3 point,
			ref int minTriangleIndex)
		{
			var result = new Vector3();
			var distanceBuf = new Vector3();
			double s = 0; double t = 0;

			var buffer = new SupportTriangle();

			double minDistance = double.MaxValue;

			for (int i = 0; i < triangles.Count; i++)
			{
				buffer = triangles[i];

				if (!GeometryUtilities.TestCollinearity(
						buffer.a.s,
						buffer.b.s,
						buffer.c.s))
				{
					distanceBuf = GeometryUtilities.GetPointTriangleIntersection(
						buffer.a.s,
						buffer.b.s,
						buffer.c.s,
						point,
						ref s,
						ref t).Value;

					buffer.SetValueS(s);
					buffer.SetValueT(t);
				}
				else
				{
					continue;
				}

				triangles[i] = buffer;

				double distance = Vector3.Length(distanceBuf);

				if (distance < minDistance)
				{
					minDistance = distance;
					minTriangleIndex = i;
					result = distanceBuf;
				}
			}

			return result;
		}

		/// <summary>
		/// Ritorna la distanza se distanza 0.0f allora vi è intersezione o compenetrazione tra gli oggetti, 
		/// se distanza 0.0 allora i due oggetti non collidono
		/// </summary>
		/// <returns>The GJK algorithm.</returns>
		/// <param name="shape1">Shape1.</param>
		/// <param name="shape2">Shape2.</param>
		/// <param name="cp">Cp.</param>
		/// <param name="isIntersection">If set to <c>true</c> is itersection.</param>
		private double ExecuteGJKAlgorithm(
			SimulationObject shape1,
			SimulationObject shape2,
			ref Vector3 collisionNormal,
			ref CollisionPoint cp,
			ref List<SupportTriangle> triangles,
			ref Vector3 centroid,
			ref bool isIntersection)
		{
			double minDistance = double.MaxValue;
			int minTriangleIndex = -1;
			var result = new EngineCollisionPoint();
			var oldDirection = new Vector3();
			var simplex = new Simplex();

			//Primo punto del simplex
			simplex.Support.Add(GetFarthestPoint(shape1, shape2));

			//Secondo punto del simplex
			Vector3 direction = Vector3.Normalize(simplex.Support[0].s * -1.0);
			if(!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(shape1, shape2, direction)))
                return -1.0;

			//Terzo punto del simplex
			direction = Vector3.Normalize(GetDirectionOnSimplex2(simplex));
			if(!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(shape1, shape2, direction)))
                return -1.0;

            	//Quarto punto del simplex
			direction = Vector3.Normalize(GeometryUtilities.CalculateNormal(
				simplex.Support[0].s,
				simplex.Support[1].s,
				simplex.Support[2].s));

			if (!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(shape1, shape2, direction)))
				simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(shape1, shape2, -1.0 * direction));

			//Costruisco il poliedro
			centroid = Helper.SetStartTriangle(
								   	ref triangles,
									simplex.Support.ToArray());

			//Verifico che l'origine sia contenuta nel poliedro
			if (Helper.IsInConvexPoly(origin, triangles))
			{
				isIntersection = true;
				return -1.0;
			}

			Vector3 triangleDistance = GetMinDistance(ref triangles, origin, ref minTriangleIndex);

			result.SetDist(triangleDistance);
			result.SetNormal(Vector3.Normalize(triangleDistance));
			Helper.GetVertexFromMinkowsky(triangles[minTriangleIndex], shape1, shape2, ref result);

			minDistance = triangleDistance.Length();

			for (int i = 0; i < MaxIterations; i++) 
			{
				direction = -1.0 * triangleDistance.Normalize();

				if (Vector3.Length(direction) < constTolerance)
				{
					direction = origin - centroid;
				}

				if (direction == oldDirection)
					break;

				oldDirection = direction;

				if (!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(shape1, shape2, direction)))
				{
					for (int j = 0; j < triangles.Count; j++)
					{
						direction = triangles[j].normal;
						if (!simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(shape1, shape2, direction)))
						{
							if (simplex.AddSupport(Helper.GetMinkowskiFarthestPoint(shape1, shape2, -1.0 * direction)))
							   break;
							
							continue;
						}
						break;
					}
				}

				triangles = Helper.AddPointToConvexPolygon(triangles, simplex.Support[simplex.Support.Count - 1], centroid);

				//Verifico che l'origine sia contenuta nel poliedro
				if (Helper.IsInConvexPoly(origin, triangles))
				{
					isIntersection = true;
					return -1.0;
				}

				triangleDistance = GetMinDistance(ref triangles, origin, ref minTriangleIndex);

				double mod = triangleDistance.Length();

				if (mod < minDistance)
				{
					result.SetDist(triangleDistance);
					result.SetNormal(Vector3.Normalize(triangleDistance));
					Helper.GetVertexFromMinkowsky(triangles[minTriangleIndex], shape1, shape2, ref result);

					minDistance = mod;
				}
			}

			collisionNormal = -1.0 * result.normal;

			cp = new CollisionPoint(
				result.a,
				result.b,
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
			SimulationObject objectA, 
			SimulationObject objectB)
		{
			var collisionPoint = new CollisionPoint();
			var collisionNormal = new Vector3();
			var supportTriangles = new List<SupportTriangle>();
			var centroid = new Vector3();
			bool isIntersection = false;

			double collisionDistance = ExecuteGJKAlgorithm (
				                          objectA,
				                          objectB,
				                          ref collisionNormal,
				                          ref collisionPoint,
				                          ref supportTriangles,
										  ref centroid,
				                          ref isIntersection);

			return new GJKOutput (
				collisionDistance,
				collisionPoint,
				collisionNormal,
				centroid,
				isIntersection,
				supportTriangles);
		}
			

		#endregion

	}
}

