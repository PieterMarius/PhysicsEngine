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
		#region Public Properties

		/// <summary>
		/// The simulation parameters.
		/// </summary>
		public SimulationParameters SimulationEngineParameters { get; private set; }

		/// <summary>
		/// Gets the collision engine parameters.
		/// </summary>
		/// <value>The collision engine parameter.</value>
		public CollisionEngineParameters CollisionEngineParam { get; private set; }

		/// <summary>
		/// Gets the solver parameters.
		/// </summary>
		/// <value>The solver parameter.</value>
		public SolverParameters SolverParam { get; private set; }

		/// <summary>
		/// The simulation objects.
		/// </summary>
		public SimulationObject[] SimulationObjects { get; private set; }

		/// <summary>
		/// The simulation joints.
		/// </summary>
		public List<IConstraint> SimulationJoints { get; private set; }

		/// <summary>
		/// Gets the time step.
		/// </summary>
		/// <value>The time step.</value>
		public double TimeStep { get; private set; }

		#endregion

		#region Private Properties

		//TODO Verificare se utilizzarlo
		/// <summary>
		/// The simulation objects Countinuos Collision Detecttion.
		/// </summary>
		SimulationObject[] simulationObjectsCCD;

		ObjectGeometry[] objectsGeometry;

		ICollisionEngine collisionEngine;

		List<CollisionPointStructure> collisionPoints;

		List<List<CollisionPointStructure>> collisionPartitionedPoints;

		List<List<IConstraint>> partitionedJoint;

		ISolver solver;

		double solverError;

		IContactPartitioningEngine contactPartitioningEngine;

		#endregion

		#region Constructor

		public PhysicsEngine (
			SimulationParameters simulationParameters,
			CollisionEngineParameters collisionEngineParameters,
			SolverParameters solverParameters)
		{
			SolverParam = solverParameters;

			solver = new ProjectedGaussSeidel(solverParameters);

			CollisionEngineParam = collisionEngineParameters;

			collisionEngine = new CollisionDetectionEngine(collisionEngineParameters);

			SimulationEngineParameters = simulationParameters;

			contactPartitioningEngine = new ContactPartitioningEngine();

			SimulationObjects = new SimulationObject[0];
			SimulationJoints = new List<IConstraint> ();
		}

		#endregion

		#region Public Methods

		public void SetSimulationParameters(SimulationParameters simulationParameters)
		{
			SimulationEngineParameters = simulationParameters;
		}

		#region Simulation Object 

		public void AddObject(SimulationObject simulationObject)
		{
			if (SimulationObjects != null && 
			    SimulationObjects.Length > 0) 
			{
				List<SimulationObject> bufferList = SimulationObjects.ToList ();
				bufferList.Add (simulationObject);
				SimulationObjects = bufferList.ToArray ();
			} 
			else 
			{
				var bufferList = new List<SimulationObject> ();
				bufferList.Add (simulationObject);
				SimulationObjects = bufferList.ToArray ();
			}
		}

		public void RemoveObject(int objectIndex)
		{
			if (SimulationObjects != null && 
			    SimulationObjects.Length > objectIndex) 
			{
				#region Remove object
				
				List<SimulationObject> bufferList = SimulationObjects.ToList ();
				bufferList.RemoveAt (objectIndex);
				SimulationObjects = bufferList.ToArray ();

				#endregion

				#region Remove Object Constraint

				if (SimulationJoints != null)
				{
					for (int i = SimulationJoints.Count - 1; i >= 0; i--)
					{
						if (SimulationJoints[i].GetObjectIndexA() == objectIndex ||
							SimulationJoints[i].GetObjectIndexB() == objectIndex)
						{
							Removejoint(i);
						}
					}

					foreach(IConstraint constraint in SimulationJoints)
					{
						if (constraint.GetObjectIndexA() > objectIndex)
							constraint.SetObjectIndexA(constraint.GetObjectIndexA() - 1);
						if (constraint.GetObjectIndexB() > objectIndex)
							constraint.SetObjectIndexB(constraint.GetObjectIndexB() - 1);
					}
				}

				#endregion
			}
		}

		public void RemoveAllObjects()
		{
			SimulationObjects = new SimulationObject[0];
			SimulationJoints = new List<IConstraint>();
		}

		public SimulationObject GetObject(int objectIndex)
		{
			return simulationObjectsCCD [objectIndex];
		}
					
		#endregion

		#region Simulation Joint

		public void AddJoint(IConstraint simulationJoint)
		{
			if (SimulationJoints != null &&
				SimulationJoints.Count > 0)
			{
				SimulationJoints.Add(simulationJoint);
			}
			else
			{
				SimulationJoints = new List<IConstraint>();
				SimulationJoints.Add(simulationJoint);
			}
		}

		public void Removejoint(int jointIndex)
		{
			if (SimulationJoints != null &&
			   SimulationJoints.Count > jointIndex)
			{
				SimulationJoints.RemoveAt(jointIndex);
			}
		}

		public void RemoveAllJoints()
		{
			SimulationJoints = new List<IConstraint>();
		}

		public IConstraint GetConstraint(int constraintId)
		{
			if (SimulationJoints != null &&
				SimulationJoints.Count > constraintId)
				return SimulationJoints[constraintId];
			
			return null;
		}

		#endregion

		#region Collision Engine

		public List<CollisionPointStructure> GetCollisionPointStrucureList()
		{
			return collisionPoints;
		}

		#endregion

		#region Solver

		public SolverParameters GetSolverParameters()
		{
			return SolverParam;
		}

		public void SetSolverParameters(SolverParameters parameters)
		{
			SolverParam = parameters;

			var type = (SolverType)Enum.Parse(typeof(SolverType), solver.GetType().Name);

			SetSolver(type);
		}

		public double GetSolverError()
		{
			return solverError;
		}

		public void SetSolver(SolverType type)
		{
			switch (type)
			{
				case SolverType.ProjectedGaussSeidel:
					solver = new ProjectedGaussSeidel(SolverParam);
					break;

				case SolverType.NonLinearConjugateGradient:
					solver = new NonLinearConjugateGradient(SolverParam);
					break;

				default:
					solver = new ProjectedGaussSeidel(SolverParam);
					break;
			}
		}

		#endregion

		#region Start Engine

		/// <summary>
		/// Runs the engine.
		/// </summary>
		public void Simulate(double? timeStep = null)
		{
			TimeStep = SimulationEngineParameters.TimeStep;

			if (timeStep.HasValue)
				TimeStep = timeStep.Value;
			
			#region Simulation Workflow

			collisionDetection ();

			partitionEngineExecute ();

			physicsExecutionFlow ();

			#endregion

			collisionPartitionedPoints = null;

			simulationObjectsCCD = new SimulationObject[SimulationObjects.Length];
			Array.Copy (SimulationObjects, simulationObjectsCCD, SimulationObjects.Length);

		}

		public void SimulateCCD()
		{
			throw new NotImplementedException();
			//			this.simulationObjectsCCD = new SimulationObject[this.simulationObjects.Length];
			//			Array.Copy (this.simulationObjects, this.simulationObjectsCCD, this.simulationObjects.Length);


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
			List<SpatialPartition> partitions = contactPartitioningEngine.calculateSpatialPartitioning (
				                                    collisionPoints,
				                                    SimulationJoints,
				                                    SimulationObjects);

			if (partitions != null) {

				collisionPartitionedPoints = new List<List<CollisionPointStructure>> ();
				partitionedJoint = new List<List<IConstraint>> ();

				for (int i = 0; i < partitions.Count; i++) 
				{
					var partitionedCollision = new List<CollisionPointStructure> ();
					var partJoint = new List<IConstraint> ();

					for (int j = 0; j < partitions [i].ObjectList.Count; j++) 
					{
						if (partitions [i].ObjectList [j].Type == ContactGroupType.Collision) 
						{

							CollisionPointStructure cpStruct = collisionPoints.Find(item =>
															   item.ObjectA == partitions[i].ObjectList[j].IndexA &&
															   item.ObjectB == partitions[i].ObjectList[j].IndexB);
							partitionedCollision.Add (cpStruct);

						} else {

							IConstraint smJoint = SimulationJoints.Find(item =>
																     item.GetObjectIndexA() == partitions[i].ObjectList[j].IndexA &&
																	 item.GetObjectIndexB() == partitions[i].ObjectList[j].IndexB);
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
			var stopwatch = new Stopwatch();

			stopwatch.Reset ();

			stopwatch.Start ();

			#region Contact and Joint elaboration

			solverError = 0.0;

			if (collisionPartitionedPoints != null) {

				for (int i = 0; i<  collisionPartitionedPoints.Count;i++)
				{
					List<JacobianContact> contactConstraints = GetJacobianConstraint (
						                                           collisionPartitionedPoints [i],
						                                           partitionedJoint [i],
						                                           SimulationObjects,
						                                           SimulationEngineParameters);

					JacobianContact[] contactArray = contactConstraints.ToArray ();

					//Build solver data
					LinearProblemProperties linearProblemProperties = buildLCPMatrix (contactArray);

					if (contactConstraints.Count > 0 &&
					    linearProblemProperties != null) 
					{

						double[] X = solver.Solve (linearProblemProperties);

						solverError += solver.GetDifferentialMSE();

						//Update Objects velocity
						updateVelocity(
							contactArray,
							X,
							SimulationObjects);
					}
				}
			}

			#endregion

			#region Position and Velocity integration

			integrateObjectsPosition (SimulationObjects);

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
			collisionPoints = new List<CollisionPointStructure> ();

			//Creo l'array contenente la geometria degli oggetti
			objectsGeometry = Array.ConvertAll (SimulationObjects, 
				item => (item.ExcludeFromCollisionDetection) ? null : item.ObjectGeometry);


			var stopwatch = new Stopwatch();

			stopwatch.Reset ();
			stopwatch.Start ();

			//Eseguo il motore che gestisce le collisioni
			collisionPoints = collisionEngine.RunTestCollision (
				objectsGeometry,
				SimulationEngineParameters.CollisionDistance);
			
			stopwatch.Stop ();

			Console.WriteLine("Collision Elapsed={0}",stopwatch.ElapsedMilliseconds);
		}

		#endregion

		#region Jacobian Constraint

		public List<JacobianContact> GetJacobianConstraint(
			List<CollisionPointStructure> collisionPointsStruct,
			List<IConstraint> simulationJointList,
			SimulationObject[] simulationObjs,
			SimulationParameters simulationParameters)
		{
			var constraint = new List<JacobianContact>();

			#region Collision Contact

			constraint.AddRange(
				ContactConstraint.BuildJoints(
					collisionPointsStruct,
					simulationObjs,
					simulationParameters));

			#endregion

			#region Joint

			foreach (IConstraintBuilder constraintItem in simulationJointList)
			{
				constraint.AddRange(
					constraintItem.BuildJacobian(simulationObjs));
			}

			#endregion

			return constraint;
		}

		#endregion

		#region Solver Matrix Computation

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
				var sync = new object ();

				Parallel.For (0, 
					contact.Length, 
					new ParallelOptions { MaxDegreeOfParallelism = SimulationEngineParameters.MaxThreadNumber }, 
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

								mValue = addLCPValue (
									contactA,
									contactB);

								if (i == j) {
									mValue += contactA.CFM + 1E-30;
									D [i] = 1.0 / mValue;
									continue;
								}

								if (Math.Abs(mValue) > 1E-100) {
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
					contactB.LinearComponentA * SimulationObjects [contactA.ObjectA].InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
					SimulationObjects [contactA.ObjectA].InertiaTensor * contactB.AngularComponentA);

			} else if (contactB.ObjectB == contactA.ObjectA) {

				linearA = contactA.LinearComponentA.Dot (
					contactB.LinearComponentB * SimulationObjects [contactA.ObjectA].InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
					SimulationObjects [contactA.ObjectA].InertiaTensor * contactB.AngularComponentB);
			}

			if (contactB.ObjectA == contactA.ObjectB) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentA * SimulationObjects [contactA.ObjectB].InverseMass);
				
				angularB = contactA.AngularComponentB.Dot(
					SimulationObjects [contactA.ObjectB].InertiaTensor * contactB.AngularComponentA);
				
			} else if (contactB.ObjectB == contactA.ObjectB) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentB * SimulationObjects [contactA.ObjectB].InverseMass);
				
				angularB = contactA.AngularComponentB.Dot (
					SimulationObjects [contactA.ObjectB].InertiaTensor * contactB.AngularComponentB);
			}

			return (linearA + angularA) + (linearB + angularB);
		}

		#endregion

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
				updateObjectVelocity(
					simulationObj,
					ct.LinearComponentA,
					ct.AngularComponentA,
					X[index],
					ct.ObjectA);

				updateObjectVelocity(
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
		/// <returns>The object velocity.</returns>
		/// <param name="simulationObj">Simulation object.</param>
		/// <param name="linearComponent">Linear component.</param>
		/// <param name="angularComponent">Angular component.</param>
		/// <param name="X">X.</param>
		/// <param name="objectIndex">Object index.</param>
		private void updateObjectVelocity(
			SimulationObject[] simulationObj,
			Vector3 linearComponent,
			Vector3 angularComponent,
			double X,
			int objectIndex)
		{
			if (SimulationObjects [objectIndex].ObjectType != ObjectType.StaticRigidBody) 
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

					Vector3 linearVelocityValue = simObj.LinearVelocity +
										     (simObj.ForceValue * simObj.InverseMass) +
										     (TimeStep * SimulationEngineParameters.ExternalForce);

					simObj.SetLinearVelocity (linearVelocityValue);
					simObj.SetForce(new Vector3());

					double linearVelocity = simObj.LinearVelocity.Length ();

					/* TODO Stabilizza l'animazione*/		
					//Velocità lineare e angolare
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
						TimeStep * 
						simObj.LinearVelocity);

					#endregion

					#region Angular Velocity

					Vector3 angularTorqueValue = simObj.AngularVelocity +
												 simObj.InertiaTensor * simObj.TorqueValue;

					simObj.SetAngularVelocity(angularTorqueValue);
					simObj.SetTorque(new Vector3());

					double angularVelocity = simObj.AngularVelocity.Length();

					Vector3 versor = simObj.AngularVelocity.Normalize ();

					//Inertia parameter
					//					angularVelocity = Math.Max (0.0, 
					//						angularVelocity + 
					//						angularVelocity * 
					//						this.simulationParameters.InertiaParameter);

					double rotationAngle = angularVelocity * TimeStep;

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
						var box = new AABB (
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
				SimulationObjects [index] = simObj;
				index++;
			}
		}

		#endregion

		#endregion

	}
}

