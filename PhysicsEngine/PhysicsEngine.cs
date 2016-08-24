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
		/// Gets the time step.
		/// </summary>
		/// <value>The time step.</value>
		public double TimeStep { get; private set; }

		#endregion

		#region Private Properties

		/// <summary>
		/// The simulation objects.
		/// </summary>
		SimulationObject[] simulationObjects;

		/// <summary>
		/// The simulation joints.
		/// </summary>
		List<IConstraint> simulationJoints;

		/// <summary>
		/// The objects geometry.
		/// </summary>
		ObjectGeometry[] objectsGeometry;

		/// <summary>
		/// The collision engine.
		/// </summary>
		ICollisionEngine collisionEngine;

		/// <summary>
		/// The collision points.
		/// </summary>
		CollisionPointStructure[] collisionPoints;

		/// <summary>
		/// The collision partitioned points.
		/// </summary>
		List<List<CollisionPointStructure>> collisionPartitionedPoints;

		/// <summary>
		/// The partitioned joint.
		/// </summary>
		List<List<IConstraint>> partitionedJoint;

		/// <summary>
		/// The solver.
		/// </summary>
		ISolver solver;

		/// <summary>
		/// The contact partitioning engine.
		/// </summary>
		IContactPartitioningEngine contactPartitioningEngine;

		double solverError;

		#endregion

		#region Constructor

		public PhysicsEngine (
			SimulationParameters simulationParameters,
			CollisionEngineParameters collisionEngineParameters,
			SolverParameters solverParameters)
		{
			SolverParam = solverParameters;

			SetSolver(SolverType.NonLinearConjugateGradient);

			CollisionEngineParam = collisionEngineParameters;

			collisionEngine = new CollisionDetectionEngine(collisionEngineParameters);

			SimulationEngineParameters = simulationParameters;

			contactPartitioningEngine = new ContactPartitioningEngine();

			simulationObjects = new SimulationObject[0];
			simulationJoints = new List<IConstraint> ();
		}

		#endregion

		#region Public Methods

		#region Simulation Object 

		public void AddObject(SimulationObject simulationObject)
		{
			if (simulationObjects != null && 
			    simulationObjects.Length > 0) 
			{
				List<SimulationObject> bufferList = simulationObjects.ToList ();
				bufferList.Add (simulationObject);
				simulationObjects = bufferList.ToArray ();
			} 
			else 
			{
				var bufferList = new List<SimulationObject> ();
				bufferList.Add (simulationObject);
				simulationObjects = bufferList.ToArray ();
			}
		}

		public void RemoveObject(int objectIndex)
		{
			if (simulationObjects != null && 
			    simulationObjects.Length > objectIndex) 
			{
				#region Remove object
				
				List<SimulationObject> bufferList = simulationObjects.ToList ();
				bufferList.RemoveAt (objectIndex);
				simulationObjects = bufferList.ToArray ();

				#endregion

				#region Remove Object Constraint

				if (simulationJoints != null)
				{
					for (int i = simulationJoints.Count - 1; i >= 0; i--)
					{
						if (simulationJoints[i].GetObjectIndexA() == objectIndex ||
							simulationJoints[i].GetObjectIndexB() == objectIndex)
						{
							RemoveJoint(i);
						}
					}

					foreach(IConstraint constraint in simulationJoints)
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
			simulationObjects = new SimulationObject[0];
			simulationJoints = new List<IConstraint>();
		}

		public SimulationObject GetObject(int objectIndex)
		{
			return simulationObjects[objectIndex];
		}

		public int ObjectCount()
		{
			return simulationObjects.Length;
		}

		public SimulationObject[] GetSimulationObjectArray()
		{
			return simulationObjects;
		}
					
		#endregion

		#region Simulation Joint

		public void AddJoint(IConstraint simulationJoint)
		{
			if (simulationJoints != null &&
				simulationJoints.Count > 0)
			{
				simulationJoints.Add(simulationJoint);
			}
			else
			{
				simulationJoints = new List<IConstraint>();
				simulationJoints.Add(simulationJoint);
			}
		}

		public void RemoveJoint(int jointIndex)
		{
			if (simulationJoints != null &&
			   simulationJoints.Count > jointIndex)
			{
				simulationJoints.RemoveAt(jointIndex);
			}
		}

		public void RemoveAllJoints()
		{
			simulationJoints = new List<IConstraint>();
		}

		public IConstraint GetJoint(int constraintId)
		{
			if (simulationJoints != null &&
				simulationJoints.Count > constraintId)
				return simulationJoints[constraintId];
			
			return null;
		}

		public int JointsCount()
		{
			return simulationJoints.Count;
		}

		public List<IConstraint> GetJointsList()
		{
			return new List<IConstraint>(simulationJoints);
		}

		#endregion

		#region Collision Engine

		public List<CollisionPointStructure> GetCollisionPointStrucureList()
		{
			if (collisionPoints == null)
				return new List<CollisionPointStructure>();
			
			return new List<CollisionPointStructure>(collisionPoints);
		}

		#endregion

		#region Solver

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

			CollisionDetectionStep ();

			PartitionEngineExecute ();

			if (SimulationEngineParameters.PositionStabilization)
			{
				bool positionUpdated = PhysicsPositionCorrection();

				if (positionUpdated)
				{
					CollisionDetectionStep();

					PartitionEngineExecute();
				}
			}

			physicsExecutionFlow ();

			#endregion
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

		private bool PhysicsPositionCorrection()
		{
			bool positionUpdated = false;

			if (collisionPartitionedPoints != null)
			{
				for (int i = 0; i < collisionPartitionedPoints.Count; i++)
				{
					JacobianContact[] contactConstraints = GetJacobianConstraint(
																   collisionPartitionedPoints[i].ToArray(),
																   partitionedJoint[i],
																   simulationObjects,
																   SimulationEngineParameters).ToArray();

					//TODO inserire tra i parametri
					int positionBasedIterations = 25;

					if (positionBasedIterations > 0)
					{
						JacobianContact[] collisionJointIntersection = Helper.FindConstraints(contactConstraints,
																					ConstraintType.Joint,
																					ConstraintType.JointLimit,
																					ConstraintType.JointMotor);

						LinearProblemProperties collisionErrorLCP = BuildLCPMatrix(
							collisionJointIntersection,
							SimulationEngineParameters.PositionStabilization,
							true);

						if (collisionErrorLCP != null)
						{
							solver.GetSolverParameters().SetSolverMaxIteration(positionBasedIterations);

							double[] correctionValues = solver.Solve(collisionErrorLCP);

							UpdatePosition(collisionJointIntersection,
										   simulationObjects,
										   correctionValues);

							positionUpdated = true;
						}
					}
				}
			}

			return positionUpdated;
		}

		private void physicsExecutionFlow()
		{
			var stopwatch = new Stopwatch();

			stopwatch.Reset();

			stopwatch.Start();

			#region Contact and Joint elaboration

			solverError = 0.0;

			if (collisionPartitionedPoints != null) {

				for (int i = 0; i < collisionPartitionedPoints.Count;i++)
				{
					JacobianContact[] contactConstraints = GetJacobianConstraint(
																   collisionPartitionedPoints[i].ToArray(),
																   partitionedJoint[i],
																   simulationObjects,
																   SimulationEngineParameters).ToArray();

					#region Solve Normal Constraints

					if (SimulationEngineParameters.NormalCollisionIterations > 0)
					{
						JacobianContact[] collisionJointContact = Helper.FindConstraints(contactConstraints,
																						 ConstraintType.Collision);

						LinearProblemProperties collisionLCP = BuildLCPMatrix(
																	collisionJointContact,
																	false,
																	false);

						BuildMatrixAndExecuteSolver(collisionJointContact,
						                            collisionLCP,
						                            SimulationEngineParameters.NormalCollisionIterations);
					}

					#endregion

					#region Solve Normal And Friction Constraints

					if (SimulationEngineParameters.FrictionAndNormalIterations > 0)
					{
						JacobianContact[] frictionConstraint = Helper.FindConstraints(contactConstraints,
																				   ConstraintType.Friction,
																				   ConstraintType.Collision);

						LinearProblemProperties frictionLCP = BuildLCPMatrix(
																frictionConstraint,
																false,
																false);

						BuildMatrixAndExecuteSolver(frictionConstraint,
													frictionLCP,
													SimulationEngineParameters.FrictionAndNormalIterations);
					}

					#endregion

					#region Solve Joint Constraint

					if (simulationJoints.Count > 0 &&
						SimulationEngineParameters.JointsIterations > 0)
					{
						JacobianContact[] jointConstraints = Helper.FindConstraints(contactConstraints,
																					ConstraintType.Joint,
																					ConstraintType.JointLimit,
																					ConstraintType.JointMotor);

						LinearProblemProperties jointLCP = BuildLCPMatrix(
																jointConstraints,
																SimulationEngineParameters.PositionStabilization,
																false);

						BuildMatrixAndExecuteSolver(jointConstraints,
													jointLCP,
													SimulationEngineParameters.JointsIterations);
					}

					#endregion

					#region Solver Overall Constraints

					LinearProblemProperties overallLCP = BuildLCPMatrix(
															contactConstraints,
															false,
															false);

					if (overallLCP != null)
					{
						solver.GetSolverParameters().SetSolverMaxIteration(SimulationEngineParameters.OverallConstraintsIterations);

						double[] overallSolution = solver.Solve(overallLCP);

						solverError += solver.GetDifferentialMSE();

						//Update Objects velocity
						UpdateVelocity(
							contactConstraints,
							simulationObjects,
							overallSolution);
					}

					#endregion
				}
			}

			#endregion

			#region Position and Velocity integration

			IntegrateObjectsPosition (simulationObjects);

			#endregion

			#region Clear Array

			collisionPartitionedPoints = null;

			#endregion

			stopwatch.Stop();

			Console.WriteLine("Inner Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

		}



		#region Collision Detection

		/// <summary>
		/// Collisions detection.
		/// </summary>
		private void CollisionDetectionStep()
		{
			#region Find New Collision Points

			//Creo l'array contenente la geometria degli oggetti
			objectsGeometry = Array.ConvertAll (simulationObjects, 
				item => (item.ExcludeFromCollisionDetection) ? null : item.ObjectGeometry);

			var stopwatch = new Stopwatch();

			stopwatch.Reset ();
			stopwatch.Start ();

			//Eseguo il motore che gestisce le collisioni
			collisionPoints = collisionEngine.Execute(
									objectsGeometry,
									SimulationEngineParameters.CollisionDistance)
                                 	.ToArray();

			#endregion
			
			stopwatch.Stop ();

			Console.WriteLine("Collision Elapsed={0}",stopwatch.ElapsedMilliseconds);
		}



		#endregion

		#region Contact Partitioning

		private void PartitionEngineExecute()
		{
			List<SpatialPartition> partitions = contactPartitioningEngine.calculateSpatialPartitioning(
													collisionPoints,
													simulationJoints,
													simulationObjects);

			if (partitions != null)
			{
				collisionPartitionedPoints = new List<List<CollisionPointStructure>>();
				partitionedJoint = new List<List<IConstraint>>();

				for (int i = 0; i < partitions.Count; i++)
				{
					var partitionedCollision = new List<CollisionPointStructure>();
					var partJoint = new List<IConstraint>();

					for (int j = 0; j < partitions[i].ObjectList.Count; j++)
					{
						if (partitions[i].ObjectList[j].Type == ContactGroupType.Collision)
						{
							CollisionPointStructure? cpStruct = Helper.Find(
								collisionPoints,
								partitions[i].ObjectList[j]);

							if (cpStruct.HasValue)
								partitionedCollision.Add(cpStruct.Value);
						}
						else
						{
							IConstraint smJoint = simulationJoints.Find(item =>
												  item.GetObjectIndexA() == partitions[i].ObjectList[j].IndexA &&
												  item.GetObjectIndexB() == partitions[i].ObjectList[j].IndexB);
							partJoint.Add(smJoint);

						}
					}
					collisionPartitionedPoints.Add(partitionedCollision);
					partitionedJoint.Add(partJoint);
				}
			}
		}

		#endregion

		#region Jacobian Constraint

		public List<JacobianContact> GetJacobianConstraint(
			CollisionPointStructure[] collisionPointsStruct,
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
				constraint.AddRange(constraintItem.BuildJacobian(simulationObjs));

			#endregion

			return constraint;
		}

		#endregion

		#region Solver Matrix Builder

		private void BuildMatrixAndExecuteSolver(
			JacobianContact[] contactConstraints,
			LinearProblemProperties linearProblemProperties,
			int nIterations)
		{
			if (linearProblemProperties != null)
			{
				solver.GetSolverParameters().SetSolverMaxIteration(nIterations);

				double[]  solutionValues = solver.Solve(linearProblemProperties);

				for (int j = 0; j < contactConstraints.Length; j++)
				{
					contactConstraints[j].StartImpulse.SetStartValue(solutionValues[j]);
				}
			}
		}

		/// <summary>
		/// Builds the LCP matrix for solver.
		/// </summary>
		private LinearProblemProperties BuildLCPMatrix(
			JacobianContact[] contact,
			bool positionStabilization,
			bool execCorrection)
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

						double cfm = contactA.CFM;
						
						if (positionStabilization)
						{
							if (execCorrection)
							{
								B[i] = contactA.CorrectionValue;
								cfm = 0.0;
							}
							else
								B[i] = -contactA.B;
						}
						else
							B[i] = -(contactA.B - contactA.CorrectionValue);
						
						X[i] = contactA.StartImpulse.StartImpulseValue;
						constraints [i] = contactA.ContactReference;
						constraintsLimit [i] = contactA.ConstraintLimit;
						constraintsType [i] = contactA.Type;

						double mValue = addLCPValue(contactA,
													contactA);

						//Diagonal value
						mValue += cfm +
								  SimulationEngineParameters.CFM +
								  1E-30;
						
						D[i] = 1.0 / mValue;
						
						for (int j = i + 1; j < contact.Length; j++) 
						{
							JacobianContact contactB = contact[j];
							
							if (contactA.ObjectA == contactB.ObjectA ||
								contactA.ObjectB == contactB.ObjectB ||
								contactA.ObjectA == contactB.ObjectB ||
								contactA.ObjectB == contactB.ObjectA)
							{
								mValue = addLCPValue(
									contactA,
									contactB);

								if (Math.Abs(mValue) > 1E-100)
								{
									lock (sync)
									{
										index[i].Add(j);
										value[i].Add(mValue);
										index[j].Add(i);
										value[j].Add(mValue);
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

			if (contactA.ObjectA == contactB.ObjectA) {

				linearA = contactA.LinearComponentA.Dot (
					contactB.LinearComponentA * simulationObjects [contactA.ObjectA].InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
					simulationObjects [contactA.ObjectA].InertiaTensor * contactB.AngularComponentA);

			} else if (contactB.ObjectB == contactA.ObjectA) {

				linearA = contactA.LinearComponentA.Dot (
					contactB.LinearComponentB * simulationObjects [contactA.ObjectA].InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
					simulationObjects [contactA.ObjectA].InertiaTensor * contactB.AngularComponentB);
			}

			double linearB = 0.0;
			double angularB = 0.0;

			if (contactB.ObjectA == contactA.ObjectB) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentA * simulationObjects [contactA.ObjectB].InverseMass);
				
				angularB = contactA.AngularComponentB.Dot(
					simulationObjects [contactA.ObjectB].InertiaTensor * contactB.AngularComponentA);
				
			} else if (contactB.ObjectB == contactA.ObjectB) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentB * simulationObjects [contactA.ObjectB].InverseMass);
				
				angularB = contactA.AngularComponentB.Dot (
					simulationObjects [contactA.ObjectB].InertiaTensor * contactB.AngularComponentB);
			}

			return (linearA + angularA) +
				   (linearB + angularB);
		}

		#endregion

		#region Integrate velocity and position
			
		/// <summary>
		/// Updates velocity of the simulations objects.
		/// </summary>
		private void UpdateVelocity(
			JacobianContact[] contact,
			SimulationObject[] simulationObj,
			double[] X)
		{
			for (int i =0; i< contact.Length;i++) 
			{
				double impulse = X [i];

				JacobianContact ct = contact[i];

				UpdateObjectVelocity(
					simulationObj,
					ct.LinearComponentA,
					ct.AngularComponentA,
					impulse,
					ct.ObjectA);

				UpdateObjectVelocity(
					simulationObj,
					ct.LinearComponentB,
					ct.AngularComponentB,
					impulse,
					ct.ObjectB);

				//ct.StartImpulse.SetStartValue (impulse * SimulationEngineParameters.WarmStartingValue);
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
		/// <param name="index">Object index.</param>
		private void UpdateObjectVelocity(
			SimulationObject[] simulationObj,
			Vector3 linearComponent,
			Vector3 angularComponent,
			double X,
			int index)
		{
			SimulationObject simObj = simulationObj[index];

			if (simObj.ObjectType != ObjectType.StaticRigidBody) 
			{
				Vector3 linearImpulse = X * linearComponent;
				Vector3 angularImpuse = X * angularComponent;

				Vector3 linearVelocity = simObj.LinearVelocity +
				                         linearImpulse * simObj.InverseMass;

				Vector3 angularVelocity = simObj.AngularVelocity +
				                          (simObj.InertiaTensor *
				                          angularImpuse);

				simulationObj [index].SetLinearVelocity (linearVelocity);
				simulationObj [index].SetAngularVelocity (angularVelocity);
			}
		}

		/// <summary>
		/// Integrates the objects position.
		/// </summary>
		private void IntegrateObjectsPosition(
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
												 simObj.InertiaTensor * 
				                                 simObj.TorqueValue;

					simObj.SetAngularVelocity(angularTorqueValue);
					simObj.SetTorque(new Vector3());

					double angularVelocity = simObj.AngularVelocity.Length();

					Vector3 versor = simObj.AngularVelocity.Normalize ();

					//Rotation inertia
					angularVelocity = Math.Max (0.0, 
						angularVelocity + 
						angularVelocity * 
						SimulationEngineParameters.InertiaParameter);

					double rotationAngle = angularVelocity * TimeStep;

					var rotationQuaternion = new Quaternion (versor, rotationAngle); 

					simObj.SetRotationStatus (
						(rotationQuaternion * simObj.RotationStatus).Normalize ());

					simObj.SetRotationMatrix (simObj.RotationStatus.ConvertToMatrix ());

					simObj.SetInertiaTensor (
						(simObj.RotationMatrix * simObj.BaseInertiaTensor) *
						simObj.RotationMatrix.Transpose ());

					#endregion

					#region Update Object Vertex Position

					if (simObj.ObjectGeometry != null &&
						(linearVelocity > 0.0 || angularVelocity > 0.0)) 
					{
						for (int j = 0; j < simObj.ObjectGeometry.VertexPosition.Length; j++) 
						{
							Vector3 relativePosition = simObj.Position + 
								(simObj.RotationMatrix * simObj.RelativePositions [j]);
							
							simObj.ObjectGeometry.SetVertexPosition (
								relativePosition,
								j);
						}

						//TODO refactoring and testing
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
				simulationObjects [index] = simObj;
				index++;
			}
		}

		#region Position Based Integration

		private void UpdatePosition(
			JacobianContact[] contact,
			SimulationObject[] simulationObj,
			double[] X)
		{
			for (int i = 0; i < contact.Length; i++)
			{
				double impulse = X[i];

				JacobianContact ct = contact[i];

				UpdateObjectPosition(
					simulationObj,
					ct.LinearComponentA,
					ct.AngularComponentA,
					impulse,
					ct.ObjectA);

				UpdateObjectPosition(
					simulationObj,
					ct.LinearComponentB,
					ct.AngularComponentB,
					impulse,
					ct.ObjectB);
			}
		}

		private void UpdateObjectPosition(
			SimulationObject[] simulationObj,
			Vector3 linearComponent,
			Vector3 angularComponent,
			double X,
			int index)
		{
			SimulationObject simObj = simulationObj[index];

			if (simObj.ObjectType != ObjectType.StaticRigidBody)
			{
				Vector3 linearImpulse = X * linearComponent;
				Vector3 angularImpuse = X * angularComponent;

				Vector3 baseLinearVelocity = linearImpulse *  
			                                 simObj.InverseMass;

				Vector3 baseAngularVelocity = simObj.InertiaTensor *
										  	  angularImpuse;

				#region Linear Velocity

				double linearVelocity = baseLinearVelocity.Length();

				simObj.SetPosition(
					simObj.Position +
					TimeStep *
					baseLinearVelocity);

				#endregion

				#region Angular Velocity

				double angularVelocity = baseAngularVelocity.Length();

				Vector3 versor = baseAngularVelocity.Normalize();

				double rotationAngle = angularVelocity * TimeStep;

				var rotationQuaternion = new Quaternion(versor, rotationAngle);

				//TODO da correggere
				simObj.SetRotationStatus(
					(rotationQuaternion * simObj.RotationStatus).Normalize());

				simObj.SetRotationMatrix(simObj.RotationStatus.ConvertToMatrix());

				simObj.SetInertiaTensor(
					(simObj.RotationMatrix * simObj.BaseInertiaTensor) *
					simObj.RotationMatrix.Transpose());

				#endregion

				#region Update Object Vertex Position

				if (simObj.ObjectGeometry != null &&
					(linearVelocity > 0.0 || angularVelocity > 0.0))
				{
					for (int j = 0; j < simObj.ObjectGeometry.VertexPosition.Length; j++)
					{
						Vector3 relativePosition = simObj.Position +
							(simObj.RotationMatrix * simObj.RelativePositions[j]);

						simObj.ObjectGeometry.SetVertexPosition(
							relativePosition,
							j);
					}

					//TODO refactoring and testing
					var box = new AABB(
						simObj.ObjectGeometry.VertexPosition.Min(point => point.x),
						simObj.ObjectGeometry.VertexPosition.Max(point => point.x),
						simObj.ObjectGeometry.VertexPosition.Min(point => point.y),
						simObj.ObjectGeometry.VertexPosition.Max(point => point.y),
						simObj.ObjectGeometry.VertexPosition.Min(point => point.z),
						simObj.ObjectGeometry.VertexPosition.Max(point => point.z),
						false);

					simObj.ObjectGeometry.SetAABB(box);
				}

				#endregion

				simulationObjects[index] = simObj;
			}
		}

		#endregion

		#endregion

		#endregion

	}
}

