using System;
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
		private List<ObjectConstraint> simulationJoints;

		#region Collision Engine

		private ICollisionEngine collisionEngine;

		private List<CollisionPointStructure> collisionPoints;

		#endregion

		#region Execution Properties

		private IJacobianConstraintBuilder jacobianConstraintBuilder;

		private List<List<CollisionPointStructure>> collisionPartitionedPoints;
		private List<List<ObjectConstraint>> partitionedJoint;

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
			this.jacobianConstraintBuilder = new JacobianConstraintBuilder ();

			this.simulationJoints = new List<ObjectConstraint> ();
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

		public void AddJoint(ObjectConstraint simulationJoint)
		{
			this.simulationJoints.Add (simulationJoint);
		}

		public void Removejoint(int jointIndex)
		{
			this.simulationJoints.RemoveAt (jointIndex);
		}

		public void RemoveAllJoints()
		{
			this.simulationJoints = new List<ObjectConstraint> ();
		}

		public ObjectConstraint GetJoint(int jointIndex)
		{
			return this.simulationJoints [jointIndex];
		}

		public List<ObjectConstraint> GetJointsList()
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

			this.collisionPartitionedPoints = null;

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
			List<SpatialPartition> partitions = this.contactPartitioningEngine.calculateSpatialPartitioning (
				                                    this.collisionPoints,
				                                    this.simulationJoints,
				                                    this.simulationObjects);

			if (partitions != null) {

				collisionPartitionedPoints = new List<List<CollisionPointStructure>> ();
				partitionedJoint = new List<List<ObjectConstraint>> ();

				for (int i = 0; i < partitions.Count; i++) 
				{
					List<CollisionPointStructure> partitionedCollision = new List<CollisionPointStructure> ();
					List<ObjectConstraint> partJoint = new List<ObjectConstraint> ();

					for (int j = 0; j < partitions [i].ObjectList.Count; j++) 
					{
						if (partitions [i].ObjectList [j].Type == ContactGroupType.Collision) 
						{

							CollisionPointStructure cpStruct = this.collisionPoints.Find (item => 
																item.ObjectA == partitions [i].ObjectList [j].IndexA &&
							                                   	item.ObjectB == partitions [i].ObjectList [j].IndexB);
							partitionedCollision.Add (cpStruct);

						} else {

							ObjectConstraint smJoint = this.simulationJoints.Find (item => 
													   item.IndexA == partitions [i].ObjectList [j].IndexA &&
							                           item.IndexB == partitions [i].ObjectList [j].IndexB);
							partJoint.Add (smJoint);

						}
					}
					collisionPartitionedPoints.Add (partitionedCollision);
					partitionedJoint.Add(partJoint);
				}
			}
		}

		#endregion

		private void physicsExecutionFlow()
		{
			Stopwatch stopwatch = new Stopwatch();

			stopwatch.Reset ();

			stopwatch.Start ();

			#region Contact and Joint elaboration

			if (this.collisionPartitionedPoints != null) {

				for (int i = 0; i<  collisionPartitionedPoints.Count;i++)
				{
					List<JacobianContact> contactConstraints = this.jacobianConstraintBuilder.GetJacobianConstraint (
						                                           this.collisionPartitionedPoints [i],
						                                           this.partitionedJoint [i],
						                                           this.simulationObjects,
						                                           this.simulationParameters);

					JacobianContact[] contactArray = contactConstraints.ToArray ();

					//Build solver data
					LinearProblemProperties linearProblemProperties = this.buildLCPMatrix (contactArray);

					if (contactConstraints.Count > 0 &&
					    linearProblemProperties != null) {

						double[] X = this.solver.Solve (linearProblemProperties);

						//Update Objects velocity
						this.updateVelocity (
							contactArray,
							X,
							this.simulationObjects);
					}
				}
			}

			#endregion

			#region Position and Velocity integration

			//Update Objects position
			this.integrateObjectsPosition (this.simulationObjects);

			//Update Joints position
			this.integrateJointPosition (this.simulationJoints);

			#endregion

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
			this.objectsGeometry = Array.ConvertAll (this.simulationObjects, 
				item => (item.ExcludeFromCollisionDetection) ? null : item.ObjectGeometry);

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
				int?[] constraints = new int?[contact.Length];

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

						B [i] = - contactA.B;
						
						X [i] = contactA.StartImpulseValue;
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

			if (contactA.ObjectA == contactB.ObjectA) {

				linearA = contactA.LinearComponentA.Dot (
					contactB.LinearComponentA * this.simulationObjects [contactA.ObjectA].InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
					this.simulationObjects [contactA.ObjectA].InertiaTensor * contactB.AngularComponentA);

			} else if (contactB.ObjectB == contactA.ObjectA) {

				linearA = contactA.LinearComponentA.Dot (
					contactB.LinearComponentB * this.simulationObjects [contactA.ObjectA].InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
					this.simulationObjects [contactA.ObjectA].InertiaTensor * contactB.AngularComponentB);
			}

			if (contactB.ObjectA == contactA.ObjectB) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentA * this.simulationObjects [contactA.ObjectB].InverseMass);
				
				angularB = contactA.AngularComponentB.Dot(
					this.simulationObjects [contactA.ObjectB].InertiaTensor * contactB.AngularComponentA);
				
			} else if (contactB.ObjectB == contactA.ObjectB) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentB * this.simulationObjects [contactA.ObjectB].InverseMass);
				
				angularB = contactA.AngularComponentB.Dot (
					this.simulationObjects [contactA.ObjectB].InertiaTensor * contactB.AngularComponentB);
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
					X [index],
					ct.ObjectA);
			
				this.updateObjectVelocity (
					simulationObj,
					ct.LinearComponentB, 
					ct.AngularComponentB,
					X [index],
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
			if (this.simulationObjects [objectIndex].ObjectType != ObjectType.StaticRigidBody) 
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
			int index = 0;
			foreach (SimulationObject simObj in simulationObj) 
			{
				
				if (simObj.ObjectType != ObjectType.StaticRigidBody) 
				{
					#region Linear Velocity

					simObj.SetLinearVelocity (
						simObj.LinearVelocity +
						this.timeStep *
						simulationParameters.ExternalForce);

					double linearVelocity = simObj.LinearVelocity.Length ();
					double angularVelocity = simObj.AngularVelocity.Length ();

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

					simObj.SetPosition (
						simObj.Position + 
						this.timeStep * 
						simObj.LinearVelocity);

					#endregion

					#region Angular Velocity

					Vector3 versor = simObj.AngularVelocity.Normalize ();

					//Inertia parameter
//					angularVelocity = Math.Max (0.0, 
//						angularVelocity + 
//						angularVelocity * 
//						this.simulationParameters.InertiaParameter);

					double rotationAngle = angularVelocity * this.timeStep;

					Quaternion rotationQuaternion = new Quaternion (versor, rotationAngle); 

					simObj.SetRotationStatus (
						(rotationQuaternion * simObj.RotationStatus).Normalize ());

					simObj.SetRotationMatrix (simObj.RotationStatus.ConvertToMatrix ());

					simObj.SetInertiaTensor (
						(simObj.RotationMatrix * simObj.BaseInertiaTensor) *
						simObj.RotationMatrix.Transpose ());

					#endregion

					#region Update Object Vertex Position

					if (simObj.ObjectGeometry != null &&
						(linearVelocity > 0.0 ||
						angularVelocity > 0.0)) 
					{
						for (int j = 0; j < simObj.ObjectGeometry.VertexPosition.Length; j++) 
						{
							Vector3 relativePosition = simObj.Position + 
								(simObj.RotationMatrix * simObj.RelativePositions [j]);
							
							simObj.ObjectGeometry.SetVertexPosition (
								relativePosition,
								j);
						}

						//TODO refactoring
						AABB box = new AABB (
							simObj.ObjectGeometry.VertexPosition.Min (point => point.x),
							simObj.ObjectGeometry.VertexPosition.Max (point => point.x),
							simObj.ObjectGeometry.VertexPosition.Min (point => point.y),
							simObj.ObjectGeometry.VertexPosition.Max (point => point.y),
							simObj.ObjectGeometry.VertexPosition.Min (point => point.z),
							simObj.ObjectGeometry.VertexPosition.Max (point => point.z),
							false);

						simObj.ObjectGeometry.SetAABB (box);
					}

					#endregion

				}
				this.simulationObjects [index] = simObj;
				index++;
			}
		}
			
		/// <summary>
		/// Integrates the joint position.
		/// </summary>
		private void integrateJointPosition(
			List<ObjectConstraint> simulationConstraints)
		{
			foreach (ObjectConstraint objConstraint in simulationConstraints) 
			{
				int indexA = objConstraint.IndexA;

				foreach (IConstraint constraintItem in objConstraint.ConstraintList) 
				{
					Vector3 relativeAnchorPosition = constraintItem.GetStartAnchorPosition () -
					                          this.simulationObjects [indexA].StartPosition;

					relativeAnchorPosition = (this.simulationObjects [indexA].RotationMatrix * relativeAnchorPosition) +
												this.simulationObjects [indexA].Position;

					constraintItem.SetAnchorPosition (relativeAnchorPosition);
				}
			}
		}

		#endregion

		#endregion

	}
}

