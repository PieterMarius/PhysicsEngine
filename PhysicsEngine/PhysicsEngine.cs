﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Threading.Tasks;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;
using CollisionEngine;
using LCPSolver;

namespace MonoPhysicsEngine
{
	public class PhysicsEngine
	{
		#region Private Properties

		/// <summary>
		/// The simulation parameters.
		/// </summary>
		private SimulationParameters simulationParameters;

		/// <summary>
		/// The simulation objects.
		/// </summary>
		private SimulationObject[] simulationObjects;

		/// <summary>
		/// The simulation objects Countinuos Collision Detecttion.
		/// </summary>
		private SimulationObject[] simulationObjectsCCD;

		/// <summary>
		/// The objects geometry.
		/// </summary>
		private ObjectGeometry[] objectsGeometry;

		/// <summary>
		/// The simulation joints.
		/// </summary>
		private List<SimulationJoint> simulationJoints;

		#region Collision Engine

		private ICollisionEngine collisionEngine;

		private List<CollisionPointStructure> collisionPoints;

		#endregion

		#region Execution Properties

		List<List<CollisionPointStructure>> collisionPartitionedPoints;
		List<List<SimulationJoint>> partitionedJoint;

		#endregion

		#region LCP properties

		private ISolver solver;

		#endregion

		#region CCD parameters

		private double timeStep;

		#endregion

		private IContactPartitioningEngine contactPartitioningEngine;

		#endregion

		#region Constructor

		public PhysicsEngine (
			ICollisionEngine collisionEngine,
			ISolver solver,
			IContactPartitioningEngine contactPartitioningEngine,
			SimulationParameters simulationParameters)
		{
			this.collisionEngine = collisionEngine;
			this.solver = solver;
			this.simulationParameters = simulationParameters;
			this.contactPartitioningEngine = contactPartitioningEngine;

			this.simulationJoints = new List<SimulationJoint> ();
		}

		#endregion

		#region Public Methods

		#region Simulation Parameters

		public SimulationParameters GetSimulationParameters()
		{
			return this.simulationParameters;
		}

		public void SetSimulationParameters(SimulationParameters simulationParameters)
		{
			this.simulationParameters = simulationParameters;
		}

		#endregion

		#region Object 

		public void AddObject(SimulationObject simulationObject)
		{
			if (this.simulationObjects != null && this.simulationObjects.Length > 0) {
				List<SimulationObject> bufferList = this.simulationObjects.ToList ();
				bufferList.Add (simulationObject);
				this.simulationObjects = bufferList.ToArray ();
			} 
			else 
			{
				List<SimulationObject> bufferList = new List<SimulationObject> ();
				bufferList.Add (simulationObject);
				this.simulationObjects = bufferList.ToArray ();
			}

		}

		public void RemoveObject(int objectIndex)
		{
			if (this.simulationObjects != null && this.simulationObjects.Length > 0) {
				List<SimulationObject> bufferList = this.simulationObjects.ToList ();
				bufferList.RemoveAt (objectIndex);
				this.simulationObjects = bufferList.ToArray ();
			}
		}

		public void RemoveAllObjects()
		{
			this.simulationObjects = null;
		}

		public SimulationObject GetObject(int objectIndex)
		{
			return this.simulationObjectsCCD [objectIndex];
		}

		public SimulationObject[] GetObjectList()
		{
			return this.simulationObjectsCCD;
		}
			
		#endregion

		#region Simulation Joint

		public void AddJoint(SimulationJoint simulationJoint)
		{
			this.simulationJoints.Add (simulationJoint);
		}

		public void Removejoint(int jointIndex)
		{
			this.simulationJoints.RemoveAt (jointIndex);
		}

		public void RemoveAllJoints()
		{
			this.simulationJoints = new List<SimulationJoint> ();
		}

		public SimulationJoint GetJoint(int jointIndex)
		{
			return this.simulationJoints [jointIndex];
		}

		public List<SimulationJoint> GetJointsList()
		{
			return this.simulationJoints;
		}

		#endregion

		#region Collision Engine

		public void SetCollisionEngine(ICollisionEngine collisionEngine)
		{
			this.collisionEngine = collisionEngine;
		}

		public List<CollisionPointStructure> GetCollisionPointStrucureList()
		{
			return this.collisionPoints;
		}

		#endregion

		#region Solver

		public void SetSolver(ISolver solver)
		{
			this.solver = solver;
		}

//		public double GetSolverError()
//		{
//			if (this.X == null || this.X.Length == 0)
//				return 0.0;
//
//			return this.solver.GetMediumSquareError (this.linearProblemProperties, this.X);
//		}

		//TODO aggiungere GetSolverErrorList

		#endregion

		#region Start Engine

		/// <summary>
		/// Runs the engine.
		/// </summary>
		public void Simulate(double? timeStep)
		{
//			this.simulationObjectsCCD = new SimulationObject[this.simulationObjects.Length];
//			Array.Copy (this.simulationObjects, this.simulationObjectsCCD, this.simulationObjects.Length);

			if (timeStep.HasValue)
				this.timeStep = timeStep.Value;
			else
				this.timeStep = this.simulationParameters.TimeStep;

			this.collisionDetection ();

			this.partitionEngineExecute ();

			this.physicsExecutionFlow ();

			this.simulationObjectsCCD = new SimulationObject[this.simulationObjects.Length];
			Array.Copy (this.simulationObjects, this.simulationObjectsCCD, this.simulationObjects.Length);

//
//			this.simulationObjectsCCD = new SimulationObject[this.simulationObjects.Length];
//			Array.Copy (this.simulationObjects, this.simulationObjectsCCD, this.simulationObjects.Length);
//			for (int k = 0; k < this.simulationObjects.Length; k++) 
//			{
//				this.simulationObjectsCCD [k] = this.simulationObjects [k];
//			}
//
//			List<CollisionPointStructure> bufferCollisionPoints;
//
//			this.timeStep = this.simulationParameters.TimeStep;
//
//			double intersectionTimeStep = 0.0;
//			double noCollisionTimeStep = 0.0;
//
//			bool exit = false;
//
//			if (this.simulationParameters.DiscreteCCD && !exit) 
//			{
//
//				for (int i = 0; i < 20; i++) 
//				{
//					//Collision detection
//					this.collisionDetection ();
//					this.extractValidCollisionPoint ();
//
//					int firstCount = this.collisionPoints.Count ();
//
//					//Set start context
//					this.physicsExecutionFlow ();
//
//					//bufferCollisionPoints = new List<CollisionPointStructure> (this.collisionPoints);
//
//					//After postion update the collision is retested
//					this.collisionDetection ();
//					this.extractValidCollisionPoint ();
//
//					//No collision detection
//					if (firstCount == 0 && 
//						this.collisionPoints.Count == 0 &&
//						i == 0)
//						break;
//
//					exit = false;
//
//					//Test for intersection
//					for (int j = 0; j < this.collisionPoints.Count; j++) 
//					{
//						Console.WriteLine ("Distance " + this.collisionPoints[j].collisionDistance);
//						//Check intersection
//						if (this.collisionPoints [j].intersection ) 
//						{
//							exit = true;
//							intersectionTimeStep = this.timeStep;
//							this.timeStep = (this.timeStep + noCollisionTimeStep) / 2.0;
//							break;
//						}
//					}
//					//No intersection, only collision
//					if (!exit)
//						break;
//
//					//Increase TimeStep (up to reference time step)
//					if (this.collisionPoints.Count == 0) 
//					{
//						noCollisionTimeStep = this.timeStep;
//						this.timeStep = (this.timeStep + intersectionTimeStep) / 2.0;
//					}
//					
//					//Array.Copy (this.simulationObjectsCCD, this.simulationObjects, this.simulationObjects.Length);
//					for (int k = 0; k < this.simulationObjects.Length; k++) 
//					{
//						this.simulationObjects [k] = this.simulationObjectsCCD [k];
//					}
//					//DEBUG
//					Console.WriteLine (this.timeStep);
//				}
//
//			}
//				
//			this.simulationObjectsCCD = new SimulationObject[this.simulationObjects.Length];
//			Array.Copy (this.simulationObjects, this.simulationObjectsCCD, this.simulationObjects.Length);
//			for (int k = 0; k < this.simulationObjects.Length; k++) 
//			{
//				this.simulationObjectsCCD [k] = this.simulationObjects [k];
//			}

		}

		#endregion

		#endregion

		#region Private Methods

		#region Contact Partitioning

		private void partitionEngineExecute()
		{
			Stopwatch stopwatch = new Stopwatch();

			stopwatch.Reset ();
			stopwatch.Start ();

			List<SpatialPartition> partitions = this.contactPartitioningEngine.calculateSpatialPartitioning (
				this.collisionPoints,
				this.simulationJoints,
				this.simulationObjects);

			if (partitions != null) {

				collisionPartitionedPoints = new List<List<CollisionPointStructure>> ();
				partitionedJoint = new List<List<SimulationJoint>> ();

				for (int i = 0; i < partitions.Count; i++) 
				{
					List<CollisionPointStructure> partitionedCollision = new List<CollisionPointStructure> ();
					List<SimulationJoint> partJoint = new List<SimulationJoint> ();

					for (int j = 0; j < partitions [i].ObjectList.Count; j++) 
					{
						if (partitions [i].ObjectList [j].Type == ContactType.Collision) 
						{

							CollisionPointStructure cpStruct = this.collisionPoints.Find (item => 
																item.ObjectA == partitions [i].ObjectList [j].IndexA &&
							                                   item.ObjectB == partitions [i].ObjectList [j].IndexB);
							partitionedCollision.Add (cpStruct);

						} else {

							SimulationJoint smJoint = this.simulationJoints.Find (item => 
													   item.IndexA == partitions [i].ObjectList [j].IndexA &&
							                           item.IndexB == partitions [i].ObjectList [j].IndexB);
							partJoint.Add (smJoint);

						}
					}
					collisionPartitionedPoints.Add (partitionedCollision);
					partitionedJoint.Add(partJoint);
				}
			}

			stopwatch.Stop ();
			Console.WriteLine("Partitioning Elapsed={0}",stopwatch.ElapsedMilliseconds);
		}

		#endregion

		private void physicsExecutionFlow()
		{
			Stopwatch stopwatch = new Stopwatch();

			stopwatch.Reset ();
			stopwatch.Start ();

			if (this.collisionPartitionedPoints != null) {

				Parallel.For (0, 
					collisionPartitionedPoints.Count, 
					new ParallelOptions { MaxDegreeOfParallelism = this.simulationParameters.MaxThreadNumber }, 
					i => {
						
						//Con i punti di collisione costruisco la matrice delle collisioni
						List<JacobianContact> contactConstraints = this.buildContactsMatrix (
							this.collisionPartitionedPoints [i],
							this.simulationObjects);

						//Costruisco la matrice dei Joints
						contactConstraints.AddRange (
							this.buildJointsMatrix (
								this.partitionedJoint[i],
								this.simulationObjects));

						JacobianContact[] contactArray = contactConstraints.ToArray ();

						//Costruisco la struttura da passare al solver
						LinearProblemProperties linearProblemProperties = this.buildLCPMatrix (contactArray);

						if (contactConstraints.Count > 0 &&
						   linearProblemProperties != null) 
						{
							double[] X = this.solver.Solve (linearProblemProperties);

							//Aggiorno la velocità degli oggetti
							this.updateVelocity (
								contactArray,
								X,
								this.simulationObjects);
						}

					});
			}

			//Aggiorno la posizione degli oggetti
			this.integrateObjectsPosition (this.simulationObjects);

			//Aggiorno la posizione dei Joint
			this.integrateJointPosition (this.simulationJoints);

			stopwatch.Stop ();
			
			Console.WriteLine ("Inner Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

		}

		#region Collision Detection

		/// <summary>
		/// Collisions detection.
		/// </summary>
		private void collisionDetection()
		{
			this.collisionPoints = new List<CollisionPointStructure> ();

			//Creo l'array contenente la geometria degli oggetti
			this.objectsGeometry = new ObjectGeometry[this.simulationObjects.Length];
			for (int i = 0; i < this.simulationObjects.Length; i++)
				objectsGeometry [i] = simulationObjects [i].ObjectGeometry;

			Stopwatch stopwatch = new Stopwatch();

			stopwatch.Reset ();
			stopwatch.Start ();

			//Eseguo il motore che gestisce le collisioni
			this.collisionPoints = this.collisionEngine.RunTestCollision (
				this.objectsGeometry,
				this.simulationParameters.CollisionDistance);
			
			stopwatch.Stop ();

			Console.WriteLine("Collision Elapsed={0}",stopwatch.ElapsedMilliseconds);
		}

		#endregion


//		private List<SpatialPartition> testPartition = new List<SpatialPartition> ();
//
//		private void calculateSpatialPartitioning()
//		{
//			if (this.collisionPoints.Count > 0) 
//			{
//				//DEBUG creo copia lista
//				List<CollisionPointStructure> collisionPointsCopy = new List<CollisionPointStructure>();
//				for (int i = 0; i < this.collisionPoints.Count; i++) 
//				{
//					collisionPointsCopy.Add (this.collisionPoints [i]);
//				}
//
//				testPartition.Clear ();
//
//				while (collisionPointsCopy.Count != 0) {
//					List<CollisionPointStructure> partition = new List<CollisionPointStructure> ();
//
//					partition.Add (collisionPointsCopy [0]);
//					recursiveSearch (
//						collisionPointsCopy [0],
//						collisionPointsCopy,
//						partition);
//
//					Console.WriteLine ("NPartition contact " + partition.Count);
//
//					for (int i = 0; i < partition.Count; i++) 
//					{
//						List<int> index = new List<int> ();
//						for (int j = 0; j < collisionPointsCopy.Count; j++) 
//						{
//							if (collisionPointsCopy [j].ObjectA == partition [i].ObjectA &&
//								collisionPointsCopy [j].ObjectB == partition [i].ObjectB) 
//							{
//								index.Add (j);
//							}
//						}
//						for (int j = 0; j < index.Count; j++) 
//						{
//							collisionPointsCopy.RemoveAt (index [j]);
//						}
//
//					}
//
//					testPartition.Add (new SpatialPartition (partition));
//				}
//			}
//
//		}
//
//		private void recursiveSearch(
//			CollisionPointStructure collisionPoint,
//			List<CollisionPointStructure> readList,
//			List<CollisionPointStructure> partition)
//		{
//			for (int i = 0; i < readList.Count; i++) {
//				if (collisionPoint.ObjectA == readList [i].ObjectA
//				    && collisionPoint.ObjectB == readList [i].ObjectB) {
//					continue;
//				} else if (this.simulationObjects [collisionPoint.ObjectA].Mass <= 0.0 &&
//				           this.simulationObjects [collisionPoint.ObjectB].Mass <= 0.0) {
//					break;
//				} else if (this.simulationObjects [collisionPoint.ObjectA].Mass <= 0.0 &&
//				           (collisionPoint.ObjectB == readList [i].ObjectA ||
//				           collisionPoint.ObjectB == readList [i].ObjectB)) {
//					
//					if (!partition.Contains (readList [i])) {
//						partition.Add (readList [i]);
//						recursiveSearch (partition [partition.Count - 1], readList, partition);
//					}
//
//				} else if (this.simulationObjects [collisionPoint.ObjectB].Mass <= 0.0 &&
//				           (collisionPoint.ObjectA == readList [i].ObjectA ||
//				           collisionPoint.ObjectA == readList [i].ObjectB)) {
//					
//					if (!partition.Contains (readList [i])) {
//						partition.Add (readList [i]);
//						recursiveSearch (partition [partition.Count - 1], readList, partition);
//					}
//
//				} else if (collisionPoint.ObjectA == readList [i].ObjectB ||
//				           collisionPoint.ObjectB == readList [i].ObjectA ||
//				           collisionPoint.ObjectA == readList [i].ObjectA ||
//				           collisionPoint.ObjectB == readList [i].ObjectB &&
//				           (this.simulationObjects [collisionPoint.ObjectA].Mass > 0.0 &&
//				           this.simulationObjects [collisionPoint.ObjectB].Mass > 0.0)) {
//
//					if (!partition.Contains (readList [i])) {
//						partition.Add (readList [i]);
//						recursiveSearch (partition [partition.Count - 1], readList, partition);
//					}
//				}
//			}	
//		}

		/// <summary>
		/// Builds the contacts matrix.
		/// </summary>
		private List<JacobianContact> buildContactsMatrix(
			List<CollisionPointStructure> collisionPointsStruct,
			SimulationObject[] simulationObjs)
		{
			List<JacobianContact> contactConstraints = new List<JacobianContact> ();

			for (int i = 0; i < collisionPointsStruct.Count; i++) 
			{
				CollisionPointStructure collisionPointStr = collisionPointsStruct [i];

				int indexA = collisionPointStr.ObjectA;
				int indexB = collisionPointStr.ObjectB;

				double restitutionCoefficient =
					1.0 + (simulationObjs [indexA].RestitutionCoeff +
						simulationObjs [indexB].RestitutionCoeff) * 0.5;
				
				for (int k = 0; k < collisionPointStr.CollisionPoints.Length; k++) 
				{
					Vector3 collisionPoint;
					if (collisionPointStr.Intersection)
						collisionPoint = collisionPointStr.CollisionPoints [k].collisionPointA;
					else
						collisionPoint = (collisionPointStr.CollisionPoints [k].collisionPointA +
							collisionPointStr.CollisionPoints [k].collisionPointB) * 0.5;

					Vector3 ra = collisionPoint - simulationObjs [indexA].Position;
					Vector3 rb = collisionPoint - simulationObjs [indexB].Position;

					Vector3 linearComponentA = Vector3.Normalize (collisionPointStr.CollisionPoints [k].collisionNormal * -1.0);

					Vector3 linearComponentB = -1.0 * linearComponentA;

					Vector3 angularComponentA = Vector3.Cross (ra, linearComponentA);

					Vector3 angularComponentB = Vector3.Cross (rb, linearComponentB);

					Vector3 velocityA = simulationObjs [indexA].LinearVelocity +
					                    Vector3.Cross (simulationObjs [indexA].AngularVelocity, ra);

					Vector3 velocityB = simulationObjs [indexB].LinearVelocity +
					                    Vector3.Cross (simulationObjs [indexB].AngularVelocity, rb);

					Vector3 relativeVelocity = velocityA - velocityB;

					Vector3 tangentialVelocity = relativeVelocity - 
						(Vector3.Dot (linearComponentA, relativeVelocity) * linearComponentA);

					#region Stabilize Animation

					if (Math.Abs (Vector3.Dot (linearComponentA, relativeVelocity)) <= 
						this.simulationParameters.VelocityToleranceStabilization)
						restitutionCoefficient = 1.0;

					#endregion

					double error = collisionPointStr.IntersectionDistance * this.simulationParameters.BaumStabilization;
					double b = Vector3.Dot (linearComponentA, relativeVelocity) * restitutionCoefficient -
					           error;

					//Normal direction force
					JacobianContact normalDirection = new JacobianContact (
						                                  indexA,
						                                  indexB,
						                                  0,
						                                  linearComponentA,
						                                  linearComponentB,
						                                  angularComponentA,
						                                  angularComponentB,
						                                  ConstraintType.Collision,
						                                  b,
						                                  0.0,
						                                  0.0);

					JacobianContact[] frictionContact;

					if (Vector3.Length (tangentialVelocity) > 
						this.simulationParameters.ShiftToStaticFrictionTolerance) 
					{
						//Dynamic friction
						frictionContact = this.addFriction (
							indexA,
							indexB,
							collisionPoint,
							linearComponentA,
							tangentialVelocity,
							ra,
							rb,
							ConstraintType.DynamicFriction);
					} 
					else 
					{
						//Static friction
						frictionContact = this.addFriction (
							indexA,
							indexB,
							collisionPoint,
							linearComponentA,
							tangentialVelocity,
							ra,
							rb,
							ConstraintType.StaticFriction);
					}
						
					contactConstraints.Add (normalDirection);
					contactConstraints.Add (frictionContact[0]);
					contactConstraints.Add (frictionContact[1]);
				}
			}
			return contactConstraints;
		}


		/// <summary>
		/// Adds the friction.
		/// </summary>
		/// <returns>The friction.</returns>
		/// <param name="indexA">Index a.</param>
		/// <param name="indexB">Index b.</param>
		/// <param name="collisionPoint">Collision point.</param>
		/// <param name="normal">Normal.</param>
		/// <param name="tangentialVelocity">Tangential velocity.</param>
		/// <param name="frictionType">Friction type.</param>
		private JacobianContact[] addFriction(
			int indexA,
			int indexB,
			Vector3 collisionPoint,
			Vector3 normal,
			Vector3 tangentialVelocity,
			Vector3 ra,
			Vector3 rb,
			ConstraintType frictionType)
		{
			JacobianContact[] friction = new JacobianContact[2];

			Vector3[] linearComponentA = new Vector3[2];
			Vector3[] linearComponentB = new Vector3[2];
			Vector3[] angularComponentA = new Vector3[2];
			Vector3[] angularComponentB = new Vector3[2];

			double constraintLimit = 0.0;
			double B1 = 0.0;
			double B2 = 0.0;

			switch (frictionType) {

			case ConstraintType.DynamicFriction:

				constraintLimit = 0.5 * (this.simulationObjects [indexA].DynamicFrictionCoeff +
				this.simulationObjects [indexB].DynamicFrictionCoeff);

				linearComponentA[0] = Vector3.Normalize (tangentialVelocity);
				linearComponentB[0] = -1.0 * linearComponentA[0];

				angularComponentA[0] = Vector3.Cross (ra, linearComponentA[0]);
				angularComponentB[0] = Vector3.Cross (rb, linearComponentB[0]);

				B1 = Vector3.Dot (linearComponentA[0], tangentialVelocity);

				linearComponentA[1] = Vector3.Normalize (Vector3.Cross (tangentialVelocity, normal));
				linearComponentB[1] = -1.0 * linearComponentA[1];

				angularComponentA[1] = Vector3.Cross (ra, linearComponentA[1]);
				angularComponentB[1] = Vector3.Cross (rb, linearComponentB[1]);

				B2 = Vector3.Dot (linearComponentA[1], tangentialVelocity);

				break;

			case ConstraintType.StaticFriction:

				constraintLimit = 0.5 * (this.simulationObjects [indexA].StaticFrictionCoeff +
					this.simulationObjects [indexB].StaticFrictionCoeff);

				linearComponentA[0] = GeometryUtilities.ProjectVectorOnPlane (normal);
				linearComponentB[0] = -1.0 * linearComponentA[0];

				angularComponentA[0] = Vector3.Cross (ra, linearComponentA[0]);
				angularComponentB[0] = Vector3.Cross (rb, linearComponentB[0]);

				linearComponentA[1] = Vector3.Normalize (Vector3.Cross (linearComponentA[0], normal));
				linearComponentB[1] = -1.0 * linearComponentA[1];

				angularComponentA[1] = Vector3.Cross (ra, linearComponentA[1]);
				angularComponentB[1] = Vector3.Cross (rb, linearComponentB[1]);

				break;	
			}
				
			friction [0] = new JacobianContact (
				indexA,
				indexB,
				-1,
				linearComponentA[0],
				linearComponentB[0],
				angularComponentA[0],
				angularComponentB[0],
				frictionType,
				B1,
				constraintLimit,
				0.0);

			friction [1] = new JacobianContact (
				indexA,
				indexB,
				-2,
				linearComponentA[1],
				linearComponentB[1],
				angularComponentA[1],
				angularComponentB[1],
				frictionType,
				B2,
				constraintLimit,
				0.0);

			return friction;
		}

		/// <summary>
		/// Builds the joints matrix.
		/// </summary>
		private List<JacobianContact> buildJointsMatrix(
			List<SimulationJoint> simulationJointList,
			SimulationObject[] simulationObj)
		{
			List<JacobianContact> contactConstraints = new List<JacobianContact> ();

			foreach (SimulationJoint simulationJoint in simulationJointList) 
			{
				int indexA = simulationJoint.IndexA;
				int indexB = simulationJoint.IndexB;

				foreach (Joint joint in simulationJoint.JointList) 
				{
					SimulationObject simulationObjectA = simulationObj [indexA];
					SimulationObject simulationObjectB = simulationObj [indexB];

					Vector3 ra = joint.Position - simulationObjectA.Position;
					Vector3 rb = joint.Position - simulationObjectB.Position;

					Vector3 velObjA = simulationObjectA.LinearVelocity +
					                 Vector3.Cross (simulationObjectA.AngularVelocity, ra);
				
					Vector3 velObjB = simulationObjectB.LinearVelocity +
					                 Vector3.Cross (simulationObjectB.AngularVelocity, rb);

					Vector3 relativeVelocity = velObjA - velObjB;

					Vector3 r1 = simulationObjectA.RotationMatrix *
					            joint.DistanceFromA;
				
					Vector3 r2 = simulationObjectB.RotationMatrix *
					            joint.DistanceFromB;

					Vector3 p1 = simulationObjectA.Position + r1;
					Vector3 p2 = simulationObjectB.Position + r2;

					Vector3 dp = (p2 - p1);


					//vector3 tx = jt[i].t[0]/*productMatrix(ob[jt[i].A].rotmatrix,jt[i].t[0])*/;
					//vector3 ty = jt[i].t[1]/*productMatrix(ob[jt[i].A].rotmatrix,jt[i].t[1])*/;
					//vector3 tz = jt[i].t[2]/*productMatrix(ob[jt[i].A].rotmatrix,jt[i].t[2])*/;

					JacobianContact Joint1 = this.setJointConstraint (
						                         indexA,
						                         indexB,
						                         joint,
						                         dp,
						                         relativeVelocity,
						                         joint.Axis1,
						                         ra,
						                         rb);

					JacobianContact Joint2 = this.setJointConstraint (
						                         indexA,
						                         indexB,                
						                         joint,
						                         dp,
						                         relativeVelocity,
						                         joint.Axis2,
						                         ra,
						                         rb);

					JacobianContact Joint3 = this.setJointConstraint (
						                         indexA,
						                         indexB,                 
						                         joint,
						                         dp,
						                         relativeVelocity,
						                         joint.Axis3,
						                         ra,
						                         rb);

					//Critical section
					contactConstraints.Add (Joint1);
					contactConstraints.Add (Joint2);
					contactConstraints.Add (Joint3);
				}

				//TODO verificare se introdurre il vincolo
//				if (jt[i].vB >= 0) {
//					//Ruoto il vettore riferito all'oggetto A e l'oggetto B
//					vector3 v1A = productMatrix(ob[jt[i].A].rotmatrix, jt[i].v1);
//					vector3 v1B = productMatrix(ob[jt[i].B].rotmatrix, jt[i].v2);
//
//					vector3 p1a = sum(ob[jt[i].A].pos, v1A);
//					vector3 p2b = sum(ob[jt[i].B].pos, v1B);
//					vector3 dp1 = mins(p2b, p1a);
//
//					buffer[3].A = jt[i].A;
//					buffer[3].B = jt[i].B;
//					buffer[3].p = jt[i].pos;
//					buffer[3].normal = v1A;
//					buffer[3].error = jt[i].K * (dot(buffer[3].normal, dp));
//					buffer[3].b = dot(buffer[3].normal, relvel) - jt[i].C * dot(buffer[3].normal, relvel) - buffer[3].error;
//					buffer[3].CFM = 0;
//					buffer[3].limitc = 0;
//					buffer[3].typelim = 3;
//					buffer[3].solution = 0;
//
//
//					buffer[4].A = jt[i].A;
//					buffer[4].B = jt[i].B;
//					buffer[4].p = jt[i].pos;
//					buffer[4].normal = v1B;
//					buffer[4].error = jt[i].K * (dot(buffer[4].normal, dp));
//					buffer[4].b = dot(buffer[4].normal, relvel) - jt[i].C * dot(buffer[4].normal, relvel) - buffer[4].error;
//					buffer[4].CFM = 0;
//					buffer[4].limitc = 0;
//					buffer[4].typelim = 3;
//					buffer[4].solution = 0;
//
//
//					ct.push_back(buffer[3]);
//					ct.push_back(buffer[4]);
//
//				}
			}
			return contactConstraints;
		}

		/// <summary>
		/// Sets the joint constraint.
		/// </summary>
		/// <returns>The joint constraint.</returns>
		/// <param name="simulationJoint">Simulation joint.</param>
		/// <param name="distanceParameter">Distance parameter.</param>
		/// <param name="relativeVelocity">Relative velocity.</param>
		/// <param name="axis">Axis.</param>
		private JacobianContact setJointConstraint(
			int indexA,
			int indexB,
			Joint joint,
			Vector3 distanceParameter,
			Vector3 relativeVelocity,
			Vector3 axis,
			Vector3 ra,
			Vector3 rb)
		{
			Vector3 linearComponentA = axis;

			Vector3 linearComponentB = -1.0 * linearComponentA;

			Vector3 angularComponentA = Vector3.Cross (ra, linearComponentA);

			Vector3 angularComponentB = Vector3.Cross (rb, linearComponentB);


			double error = joint.K * Vector3.Dot (axis, distanceParameter);
			double B = Vector3.Dot (axis, relativeVelocity) -
			           joint.C * Vector3.Dot (axis, relativeVelocity) -
			           error;

			return new JacobianContact (
				indexA,
				indexB,
				-1,
				linearComponentA,
				linearComponentB,
				angularComponentA,
				angularComponentB,
				ConstraintType.Joint,
				B,
				0.0,
				0.0);
		}

		/// <summary>
		/// Builds the LCP matrix for solver.
		/// </summary>
		private LinearProblemProperties buildLCPMatrix(
			JacobianContact[] contact)
		{
			if (contact.Length > 0) 
			{
				SparseElement[] M = new SparseElement[contact.Length];
				double[] B = new double[contact.Length];
				double[] X = new double[contact.Length];
				double[] D = new double[contact.Length];
				ConstraintType[] constraintsType = new ConstraintType[contact.Length];
				double[] constraintsLimit = new double[contact.Length];
				int[] constraints = new int[contact.Length];

				List<int>[] index = new List<int>[contact.Length];
				List<double>[] value = new List<double>[contact.Length];
				for (int i = 0; i < contact.Length; i++) 
				{
					index [i] = new List<int> ();
					value [i] = new List<double> ();
				}

				//Critical section variable
				Object sync = new object ();

				Parallel.For (0, 
					contact.Length, 
					new ParallelOptions { MaxDegreeOfParallelism = this.simulationParameters.MaxThreadNumber }, 
					i => {

						JacobianContact contactA = contact [i];

						B [i] = -contactA.B;
						
						X [i] = contactA.Solution;
						constraints [i] = contactA.ContactReference;
						constraintsLimit [i] = contactA.ConstraintLimit;
						constraintsType [i] = contactA.Type;

						double mValue;

						for (int j = i; j < contact.Length; j++) {

							JacobianContact contactB = contact [j];

							if (contactA.ObjectA == contactB.ObjectA ||
							    contactA.ObjectB == contactB.ObjectB ||
							    contactA.ObjectA == contactB.ObjectB ||
							    contactA.ObjectB == contactB.ObjectA) {

								mValue = this.addLCPValue (
									contactA,
									contactB);

								if (i == j) {
									mValue += simulationParameters.CFM + 1E-30;
									D [i] = 1.0 / mValue;
									continue;
								}

								if (mValue != 0.0) {
									lock (sync) {
										index [i].Add (j);
										value [i].Add (mValue);
										index [j].Add (i);
										value [j].Add (mValue);
									}
								}
							}
						}
					});

				for (int i = 0; i < contact.Length; i++) 
				{
					M [i] = new SparseElement (
						value [i].ToArray (),
						index [i].ToArray ());
				}

				return new LinearProblemProperties (
					M,
					B,
					X,
					D,
					constraintsLimit,
					constraintsType,
					constraints,
					contact.Length);
			}

			return null;
		}


		private double addLCPValue(
			JacobianContact contactA,
			JacobianContact contactB)
		{
			double linearA = 0.0;
			double angularA = 0.0;

			double linearB = 0.0;
			double angularB = 0.0;

			if (this.simulationObjects [contactA.ObjectA].Mass > 0.0) {

				if (contactA.ObjectA == contactB.ObjectA) {
					
					Vector3 forceOnA = contactB.LinearComponentA;
					Vector3 torqueOnA = contactB.AngularComponentA;

					linearA = Vector3.Dot (contactA.LinearComponentA, 
						forceOnA * this.simulationObjects [contactA.ObjectA].InverseMass);
					
					angularA = Vector3.Dot (contactA.AngularComponentA,
						this.simulationObjects [contactA.ObjectA].InertiaTensor * torqueOnA);

				} else if (contactB.ObjectB == contactA.ObjectA) {
					
					Vector3 forceOnA = contactB.LinearComponentB;
					Vector3 torqueOnA = contactB.AngularComponentB;

					linearA = Vector3.Dot (contactA.LinearComponentA,
						forceOnA * this.simulationObjects [contactA.ObjectA].InverseMass);
					
					angularA = Vector3.Dot (contactA.AngularComponentA,
							this.simulationObjects [contactA.ObjectA].InertiaTensor * torqueOnA);
				}
			}

			if (this.simulationObjects [contactA.ObjectB].Mass > 0.0) {

				if (contactB.ObjectA == contactA.ObjectB) {
					
					Vector3 forceOnB = contactB.LinearComponentA;
					Vector3 torqueOnB = contactB.AngularComponentA;

					linearB = Vector3.Dot (contactA.LinearComponentB, 
						forceOnB * this.simulationObjects [contactA.ObjectB].InverseMass);
					
					angularB = Vector3.Dot (contactA.AngularComponentB,
							this.simulationObjects [contactA.ObjectB].InertiaTensor * torqueOnB);
					
				} else if (contactB.ObjectB == contactA.ObjectB) {

					Vector3 forceOnB = contactB.LinearComponentB;
					Vector3 torqueOnB = contactB.AngularComponentB;

					linearB = Vector3.Dot (contactA.LinearComponentB, 
						forceOnB * this.simulationObjects [contactA.ObjectB].InverseMass);
					
					angularB = Vector3.Dot (contactA.AngularComponentB,
							this.simulationObjects [contactA.ObjectB].InertiaTensor * torqueOnB);
				}
			}

			return (linearA + angularA) + (linearB + angularB);
		}

		#region Integrate velocity and position
			
		/// <summary>
		/// Updates velocity of the simulations objects.
		/// </summary>
		private void updateVelocity(
			JacobianContact[] contact,
			double[] X,
			SimulationObject[] simulationObj)
		{
			int index = 0;
			foreach (JacobianContact ct in contact) 
			{
				this.updateObjectVelocity (
					simulationObj,
					ct.LinearComponentA, 
					ct.AngularComponentA,
					X[index],
					ct.ObjectA);
			
				this.updateObjectVelocity ( 
					simulationObj,
					ct.LinearComponentB, 
					ct.AngularComponentB,
					X[index],
					ct.ObjectB);
				
				index++;
			}
		}

		/// <summary>
		/// Updates the object velocity.
		/// </summary>
		/// <param name="index">Index.</param>
		/// <param name="normal">Normal.</param>
		/// <param name="collisionPoint">Collision point.</param>
		private void updateObjectVelocity(
			SimulationObject[] simulationObj,
			Vector3 linearComponent,
			Vector3 angularComponent,
			double X,
			int objectIndex)
		{
			if (this.simulationObjects [objectIndex].Mass > 0.0) 
			{
				Vector3 linearImpulse = X * linearComponent;
				Vector3 angularImpuse = X * angularComponent;

				Vector3 linearVelocity = simulationObj [objectIndex].LinearVelocity +
					linearImpulse * simulationObj [objectIndex].InverseMass;

				Vector3 angularVelocity = simulationObj [objectIndex].AngularVelocity +
				                          (simulationObj [objectIndex].InertiaTensor *
				                          angularImpuse);

				simulationObj [objectIndex].SetLinearVelocity (linearVelocity);
				simulationObj [objectIndex].SetAngularVelocity (angularVelocity);
			}
		}

		/// <summary>
		/// Integrates the objects position.
		/// </summary>
		private void integrateObjectsPosition(
			SimulationObject[] simulationObj)
		{
			Vector3 externalVelocityStep = this.timeStep * 
				simulationParameters.ExternalForce;

			int index = 0;
			foreach (SimulationObject simObj in simulationObj) 
			{
				if (simObj.Mass > 0.0) 
				{
					simObj.SetLinearVelocity (
						simObj.LinearVelocity + externalVelocityStep);

					/* TODO Stabilizza l'animazione*/		
					//Velocit� lineare e angolare
//					if(lengthw(ob[i].vel)<=smp.lineardisable) {
//						if(ob[i].count<smp.stabIter) ob[i].count++;
//						else if(ob[i].count==smp.stabIter) {
//							ob[i].vel=turnZero();
//						}
//					}else ob[i].count=0;
//
//					if(lengthw(ob[i].a_vel)<=smp.angulardisable) {
//						if(ob[i].a_count<smp.stabIter) ob[i].a_count++;
//						else if(ob[i].a_count==smp.stabIter) {
//							ob[i].a_vel=turnZero();
//						}
//					}else ob[i].a_count=0;

					double linearVelocity = Vector3.Length (simObj.LinearVelocity);
					double angularVelocity = Vector3.Length (simObj.AngularVelocity);

					if (linearVelocity != 0.0) 
					{
						simObj.SetPosition (
							simObj.Position + 
							this.timeStep * simObj.LinearVelocity);
					}
					if (angularVelocity != 0.0) 
					{
						Vector3 versor = Vector3.Normalize (simObj.AngularVelocity);

						//Inertia parameter
						angularVelocity = Math.Max (0.0, angularVelocity + angularVelocity * this.simulationParameters.InertiaParameter);

						double rotationAngle = angularVelocity * this.timeStep;

						Quaternion rotationQuaternion = new Quaternion (versor, rotationAngle); 

						simObj.SetRotationStatus (
							Quaternion.Normalize (rotationQuaternion * simObj.RotationStatus));

						simObj.SetRotationMatrix (
							Quaternion.ConvertToMatrix (simObj.RotationStatus));

						simObj.SetInertiaTensor (
							(simObj.RotationMatrix * simObj.BaseInertiaTensor) *
							Matrix3x3.Transpose (simObj.RotationMatrix));
					}

					//Update Simulation Object Vertex Position
					if (linearVelocity != 0.0 ||
					    angularVelocity != 0.0) 
					{
						for (int j = 0; j < simObj.ObjectGeometry.NVertex; j++) 
						{
							Vector3 relativePosition = simObj.RotationMatrix * simObj.RelativePositions [j];
							simObj.ObjectGeometry.SetVertexPosition (
								relativePosition + simObj.Position,
								j);
						}
					}
				}
				this.simulationObjects [index] = simObj;
				index++;
			}
		}
			
		/// <summary>
		/// Integrates the joint position.
		/// </summary>
		private void integrateJointPosition(
			List<SimulationJoint> simulationJoints)
		{
			for (int i = 0; i < simulationJoints.Count; i++) 
			{
				int indexA = simulationJoints [i].IndexA;

				List<Joint> joint = new List<Joint> ();

				for (int j = 0; j < simulationJoints [i].JointList.Length; j++) 
				{
					Vector3 relativePosition = simulationJoints [i].JointList[j].StartJointPos -
					                          this.simulationObjects [indexA].StartPosition;

					relativePosition = (this.simulationObjects [indexA].RotationMatrix * relativePosition) +
					this.simulationObjects [indexA].Position;

					Joint jointBuf = new Joint (
						                 simulationJoints [i].JointList [j].K,
						                 simulationJoints [i].JointList [j].C,
						                 simulationJoints [i].JointList [j].StartJointPos,
						                 relativePosition,
						                 simulationJoints [i].JointList [j].Axis1,
						                 simulationJoints [i].JointList [j].Axis2,
						                 simulationJoints [i].JointList [j].Axis3,
						                 simulationJoints [i].JointList [j].DistanceFromA,
						                 simulationJoints [i].JointList [j].DistanceFromB,
						                 simulationJoints [i].JointList [j].RotationConstraintA,
						                 simulationJoints [i].JointList [j].RotationConstraintB);

					joint.Add (jointBuf);
				}

				simulationJoints [i] = new SimulationJoint (
					simulationJoints [i].IndexA,
					simulationJoints [i].IndexB,
					joint.ToArray ());
			}
		}

		#endregion

		#endregion

	}
}

