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
	public class PhysicsEngine: IDisposable
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

        public List<List<CollisionPointStructure>> GetPartitionedCollisionPoints()
        {
            return collisionPartitionedPoints;
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

			PhysicsExecutionFlow ();

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

        private bool PhysicsJointPositionCorrection()
        {
            bool positionUpdated = false;

            if (simulationJoints.Count > 0)
            {
                if (collisionPartitionedPoints != null)
                {
                    double baumgarteStabilizationValue = 1.0 / TimeStep;

                    for (int i = 0; i < collisionPartitionedPoints.Count; i++)
                    {
                        if (SimulationEngineParameters.PositionBasedJointIterations > 0)
                        {
                            JacobianContact[] jointConstraints = GetJacobianJointConstraint(
                                                                       partitionedJoint[i],
                                                                       simulationObjects,
                                                                       baumgarteStabilizationValue).ToArray();

                            LinearProblemProperties collisionErrorLCP = BuildLCPMatrix(
                                jointConstraints,
                                SimulationEngineParameters.PositionStabilization);

                            if (collisionErrorLCP != null)
                            {
                                solver.GetSolverParameters().SetSolverMaxIteration(SimulationEngineParameters.PositionBasedJointIterations);

                                SolutionValues[] correctionValues = solver.Solve(collisionErrorLCP);

                                UpdatePositionBasedVelocity(
                                               jointConstraints,
                                               simulationObjects,
                                               correctionValues);

                                positionUpdated = true;
                            }
                        }
                    }
                    #region Position and Velocity integration

                    if (positionUpdated)
                        UpdateObjectPosition(simulationObjects);

                    #endregion
                }
            }

            return positionUpdated;
        }

        private JacobianContact[] ContactSorting(JacobianContact[] jacobianContact)
        {
            var sorted = jacobianContact.Select((x, i) => new KeyValuePair<JacobianContact, int>(x, i)).
                OrderBy(x => Math.Abs(x.Key.B)).ToArray();

            int[] sortedIndex = sorted.Select(x => x.Value).ToArray();
            JacobianContact[] sortedContact = sorted.Select(x => x.Key).ToArray();

            int[] randomIndex = new int[jacobianContact.Length];
            for (int i = 0; i < randomIndex.Length; i++)
                randomIndex[sortedIndex[i]] = i;

            for (int i = 0; i < sortedContact.Length; i++)
            {
                if (sortedContact[i].Type == ConstraintType.Friction)
                    sortedContact[i].SetContactReference(randomIndex[sortedContact[i].ContactReference.Value]);
            }

            return sortedContact;
        }

		private void PhysicsExecutionFlow()
		{
			var stopwatch = new Stopwatch();

			stopwatch.Reset();

			stopwatch.Start();

			#region Contact and Joint elaboration

			solverError = 0.0;

            if (SimulationEngineParameters.PositionStabilization)
            {
                bool positionUpdated = PhysicsJointPositionCorrection();

                if (positionUpdated)
                {
                    CollisionDetectionStep();
                    PartitionEngineExecute();
                }
            }
            
            if (collisionPartitionedPoints != null) 
			{
                bool convertSetting = false;
                if (SimulationEngineParameters.PositionStabilization)
                {
                    SimulationEngineParameters.SetPositionStabilization(false);
                    convertSetting = true;
                }

                for (int i = 0; i < collisionPartitionedPoints.Count;i++)
				{
                    JacobianContact[] jacobianConstraints = GetJacobianConstraint(
                                                                   collisionPartitionedPoints[i].ToArray(),
																   partitionedJoint[i],
																   simulationObjects,
																   SimulationEngineParameters).ToArray();

                    if (jacobianConstraints.Length > 0)
                    {
                        SolutionValues[] overallSolution = new SolutionValues[jacobianConstraints.Length];

                        #region Solve Normal And Friction Constraints

                        if (SimulationEngineParameters.FrictionAndNormalIterations > 0)
                        {
                            JacobianContact[] frictionConstraint = Helper.FilterConstraints(jacobianConstraints,
                                                                                       ConstraintType.Friction,
                                                                                       ConstraintType.Collision);

                            LinearProblemProperties frictionLCP = BuildLCPMatrix(
                                                                    frictionConstraint,
                                                                    SimulationEngineParameters.PositionStabilization);

                            SolutionValues[] contactSolution = BuildMatrixAndExecuteSolver(
                                                        frictionConstraint,
                                                        frictionLCP,
                                                        SimulationEngineParameters.FrictionAndNormalIterations);
                        }

                        #endregion

                        #region Solve Joint Constraint

                        if (simulationJoints.Count > 0 &&
                            SimulationEngineParameters.JointsIterations > 0)
                        {
                            JacobianContact[] jointConstraints = Helper.FindJointConstraints(jacobianConstraints);

                            LinearProblemProperties jointLCP = BuildLCPMatrix(
                                                                    jointConstraints,
                                                                    SimulationEngineParameters.PositionStabilization);

                            SolutionValues[] jointSolution = BuildMatrixAndExecuteSolver(
                                                        jointConstraints,
                                                        jointLCP,
                                                        SimulationEngineParameters.JointsIterations);
                        }

                        #endregion

                        #region Solver Overall Constraints

                        jacobianConstraints = ContactSorting(jacobianConstraints);

                        LinearProblemProperties overallLCP = BuildLCPMatrix(
                                                                jacobianConstraints,
                                                                SimulationEngineParameters.PositionStabilization);

                        if (overallLCP != null &&
                           SimulationEngineParameters.OverallConstraintsIterations > 0)
                        {
                            solver.GetSolverParameters().SetSolverMaxIteration(SimulationEngineParameters.OverallConstraintsIterations);

                            overallSolution = solver.Solve(overallLCP);

                            double[] overallError = new double[overallLCP.Count];

                            Console.WriteLine("error " + SolverHelper.ComputeSolverError(overallLCP, overallSolution));
                        }
                        else if (SimulationEngineParameters.OverallConstraintsIterations == 0)
                        {
                            for (int j = 0; j < overallSolution.Length; j++)
                                overallSolution[j].X = jacobianConstraints[j].StartImpulse.StartImpulseValue;
                        }

                        UpdateVelocity(
                                jacobianConstraints,
                                simulationObjects,
                                overallSolution);

                        #endregion
                    }
                }
                if (convertSetting)
                {
                    SimulationEngineParameters.SetPositionStabilization(true);
                }
            }

            #endregion

            #region Position and Velocity integration

            IntegrateObjectsPosition (simulationObjects);

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
			#region Init WarmStarting

			List<CollisionPointStructure> collisionPointsBuffer = null;

			if (collisionPoints != null &&
				collisionPoints.Length > 0)
				collisionPointsBuffer = new List<CollisionPointStructure>(collisionPoints);

			#endregion

			#region Find New Collision Points

			//Creo l'array contenente la geometria degli oggetti
			SimulationObject[] simObjects = Array.ConvertAll (
							simulationObjects, 
							item => (item.ExcludeFromCollisionDetection) ? null : item);

			var stopwatch = new Stopwatch();

			stopwatch.Reset ();
			stopwatch.Start ();

			//Eseguo il motore che gestisce le collisioni
			collisionPoints = collisionEngine.Execute(
                                    simObjects,
									SimulationEngineParameters.CollisionDistance)
                                 	.ToArray();

            #endregion

            #region WarmStarting

            //if (collisionPointsBuffer != null &&
            //    collisionPointsBuffer.Count > 0)
            //    WarmStarting(collisionPointsBuffer);

            #endregion

            stopwatch.Stop ();

			Console.WriteLine("Collision Elapsed={0}",stopwatch.ElapsedMilliseconds);
		}

		private void WarmStarting(List<CollisionPointStructure> collisionPointsBuffer)
		{
			foreach (CollisionPointStructure cPoint in collisionPointsBuffer)
			{
                //TODO Work in progress
                int pointBufferIndex = collisionPoints.ToList().FindIndex(
                                      x => (x.ObjectA == cPoint.ObjectA &&
                                            x.ObjectB == cPoint.ObjectB) ||
                                           (x.ObjectA == cPoint.ObjectB &&
                                            x.ObjectB == cPoint.ObjectA));

                

				if (pointBufferIndex > -1)
				{
					CollisionPointStructure pointBuffer = collisionPoints[pointBufferIndex];

                    if((pointBuffer.CollisionPoint.CollisionPointA - cPoint.CollisionPoint.CollisionPointA).Length() < 
                        0.001)
                    {
                        if (cPoint.FrameCount > 5)
                        {
                            collisionPoints[pointBufferIndex].CollisionPoint = cPoint.CollisionPoint;
                            collisionPoints[pointBufferIndex].CollisionPoints = cPoint.CollisionPoints;
                            //collisionPoints[pointBufferIndex].SetIntersection(cPoint.Intersection);
                            //collisionPoints[pointBufferIndex].SetObjectDistance(cPoint.ObjectDistance);
                        }

                        collisionPoints[pointBufferIndex].SetFrameCount(cPoint.FrameCount + 1);
                    }
                    else
                    {
                        collisionPoints[pointBufferIndex].SetFrameCount(0);
                    }


     //               for (int i = 0; i < pointBuffer.CollisionPoints.Count(); i++)
					//{
					//	int ppBuffer = cPoint.CollisionPoints.ToList().FindIndex(x => 	(Vector3.Length(x.CollisionPointA -
					//																			pointBuffer.CollisionPoints[i].CollisionPointA) < 0.01 &&
					//																	Vector3.Length(x.CollisionPointB -
					//																			pointBuffer.CollisionPoints[i].CollisionPointB) < 0.01) ||
					//																	(Vector3.Length(x.CollisionPointA -
					//																			pointBuffer.CollisionPoints[i].CollisionPointB) < 0.01 &&
					//																	Vector3.Length(x.CollisionPointB -
				 //                                                                               pointBuffer.CollisionPoints[i].CollisionPointA) < 0.01));

					//	if (ppBuffer > -1)
					//	{
					//		collisionPoints[pointBufferIndex].CollisionPoints[i].StartImpulseValue[0].SetStartValue(cPoint.CollisionPoints[ppBuffer].StartImpulseValue[0].StartImpulseValue);
					//		collisionPoints[pointBufferIndex].CollisionPoints[i].StartImpulseValue[1].SetStartValue(cPoint.CollisionPoints[ppBuffer].StartImpulseValue[1].StartImpulseValue);
					//		collisionPoints[pointBufferIndex].CollisionPoints[i].StartImpulseValue[2].SetStartValue(cPoint.CollisionPoints[ppBuffer].StartImpulseValue[2].StartImpulseValue);
					//	}
					//}
				}
			}
		}

		#endregion

		#region Contact Partitioning

		private void PartitionEngineExecute()
		{
            #region Clear Array

            collisionPartitionedPoints = null;

            #endregion

            List<SpatialPartition> partitions = contactPartitioningEngine.CalculateSpatialPartitioning(
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
							CollisionPointStructure cpStruct = Helper.Find(
								collisionPoints,
								partitions[i].ObjectList[j]);
                                                        
							if (cpStruct != null)
								partitionedCollision.Add(cpStruct);
						}
						else
						{
							IConstraint smJoint = simulationJoints.Find(item =>
												  item.GetObjectIndexA() == partitions[i].ObjectList[j].IndexA &&
												  item.GetObjectIndexB() == partitions[i].ObjectList[j].IndexB &&
	                                              item.GetKeyIndex() == partitions[i].ObjectList[j].KeyIndex);
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

		public List<JacobianContact> GetJacobianJointConstraint(
			List<IConstraint> simulationJointList,
			SimulationObject[] simulationObjs,
			double? stabilizationCoeff = null)
		{
			var constraint = new List<JacobianContact>();
            			
			if (stabilizationCoeff.HasValue)
			{
                	foreach (IConstraintBuilder constraintItem in simulationJointList)
				{
					constraint.AddRange(constraintItem.BuildJacobian(simulationObjs, stabilizationCoeff));
				}

			}
			else
			{
				foreach (IConstraintBuilder constraintItem in simulationJointList)
					constraint.AddRange(constraintItem.BuildJacobian(simulationObjs));
			}

			return constraint;
		}

		#endregion

		#region Solver Matrix Builder

		private SolutionValues[] BuildMatrixAndExecuteSolver(
			JacobianContact[] contactConstraints,
			LinearProblemProperties linearProblemProperties,
			int nIterations)
		{
			if (linearProblemProperties != null)
			{
				solver.GetSolverParameters().SetSolverMaxIteration(nIterations);

				SolutionValues[]  solutionValues = solver.Solve(linearProblemProperties);

				for (int j = 0; j < contactConstraints.Length; j++)
				{
					contactConstraints[j].StartImpulse.SetStartValue(solutionValues[j].X);
				}

				return solutionValues;
			}

			return null;
		}

		/// <summary>
		/// Builds the LCP matrix for solver.
		/// </summary>
		private LinearProblemProperties BuildLCPMatrix(
			JacobianContact[] contact,
			bool positionStabilization = false)
		{
			if (contact.Length > 0) 
			{
				SparseElement[] M = new SparseElement[contact.Length];
				double[] B = new double[contact.Length];
				SolutionValues[] X = new SolutionValues[contact.Length];
				double[] D = new double[contact.Length];
				ConstraintType[] constraintsType = new ConstraintType[contact.Length];
				double[] constraintsLimit = new double[contact.Length];
				List<int?>[] constraints = new List<int?>[contact.Length];
                
				List<int>[] index = new List<int>[contact.Length];
				List<double>[] value = new List<double>[contact.Length];

				for (int i = 0; i < contact.Length; i++) 
				{
					index [i] = new List<int> ();
					value [i] = new List<double> ();
                    constraints[i] = new List<int?>();
                }

				//Critical section variable
				var sync = new object ();

				Parallel.For (0, 
					contact.Length, 
					new ParallelOptions { MaxDegreeOfParallelism = SimulationEngineParameters.MaxThreadNumber }, 
					i => {

						JacobianContact contactA = contact [i];

						if (positionStabilization)
							B[i] = contactA.CorrectionValue;
						else
							B[i] = -(contactA.B - ((contactA.CorrectionValue) < 0 ? Math.Max(contactA.CorrectionValue, -SimulationEngineParameters.MaxCorrectionValue):
                                                                                    Math.Min(contactA.CorrectionValue, SimulationEngineParameters.MaxCorrectionValue)));
						
						X[i].X = contactA.StartImpulse.StartImpulseValue;

                        
                        if (contactA.ContactReference.HasValue)
                            constraints[i].Add(contactA.ContactReference);
                        
						constraintsLimit [i] = contactA.ConstraintLimit;
						constraintsType [i] = contactA.Type;
                        
						double mValue = addLCPValue(contactA,
													contactA);

						//Diagonal value
						mValue += contactA.CFM +
								  SimulationEngineParameters.CFM +
								  1E-40;

                        D[i] = 1.0 / mValue;

                        	for (int j = i + 1; j < contact.Length; j++) 
						{
							JacobianContact contactB = contact[j];
							
							if (contactA.ObjectA == contactB.ObjectA ||
								contactA.ObjectB == contactB.ObjectB ||
								contactA.ObjectA == contactB.ObjectB ||
								contactA.ObjectB == contactB.ObjectA)
							{
                                if (contactA.Type == contactB.Type && 
                                    contactB.Type == ConstraintType.Collision && 
                                    contactA.ObjectA == contactB.ObjectA && 
                                    contactA.ObjectB == contactB.ObjectB)
                                {
                                    constraints[i].Add(j);
                                    constraints[j].Add(i);
                                }

								mValue = addLCPValue(
									contactA,
									contactB);
                                
								if (Math.Abs(mValue) > 1E-30)
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


                int?[][] constraintsArray = new int?[contact.Length][];
                for (int i = 0; i < contact.Length; i++) 
				{
					M [i] = new SparseElement (
						value [i].ToArray (),
						index [i].ToArray (),
                        contact.Length);

                    constraintsArray[i] = constraints[i].ToArray();
				}
                

				return new LinearProblemProperties (
					M,
					B,
					X,
					D,
					constraintsLimit,
					constraintsType,
                    constraintsArray,
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
			SolutionValues[] X)
		{
			for (int i =0; i< contact.Length;i++) 
			{
                if (Math.Abs(X[i].X) > 1E-50)
                {
                    double impulse = X[i].X;

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

                    ct.StartImpulse.SetStartValue(impulse * SimulationEngineParameters.WarmStartingValue);
                }
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
				                         linearImpulse * 
		                                 simObj.InverseMass;

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
                    
                    	simObj.SetPosition (
						simObj.Position + 
						TimeStep * 
						simObj.LinearVelocity);

                    simObj.SetLinearVelocity(simObj.LinearVelocity +
                        (simObj.ForceValue * simObj.InverseMass) +
                        (TimeStep * SimulationEngineParameters.ExternalForce));

                    simObj.SetForce(new Vector3());

                    double linearVelocity = simObj.LinearVelocity.Length();

                    #endregion

                    #region Angular Velocity

                    double angularVelocity = simObj.AngularVelocity.Length();

                    Vector3 versor = simObj.AngularVelocity.Normalize ();

					double rotationAngle = angularVelocity * TimeStep;

                    var rotationQuaternion = new Quaternion(versor, rotationAngle);

                    	simObj.SetRotationStatus ((rotationQuaternion * simObj.RotationStatus).Normalize());

					simObj.SetRotationMatrix (simObj.RotationStatus.ConvertToMatrix ());

					simObj.SetInertiaTensor (
						(simObj.RotationMatrix * simObj.BaseInertiaTensor) *
						simObj.RotationMatrix.Transpose ());

                    simObj.SetAngularVelocity(simObj.AngularVelocity +
                                              simObj.InertiaTensor *
                                              simObj.TorqueValue);

                    angularVelocity = simObj.AngularVelocity.Length();
                    simObj.SetTorque(new Vector3());

                    #endregion

                    #region Sleeping Object

                    if (SimulationEngineParameters.SleepingObject)
                        ObjectSleep(simObj);

                    #endregion

                    #region Update AABB

                    if (simObj.ObjectGeometry != null &&
						(linearVelocity > 0.0 || angularVelocity > 0.0))
                    {
                       simObj.ObjectGeometry.SetAABB(Helper.UpdateAABB(simObj));
                    }

                    #endregion

                }
				simulationObjects [index] = simObj;
				index++;
			}
		}

        private void ObjectSleep(SimulationObject simulationObj)
        {
            if (simulationObj.LinearVelocity.Length() <= SimulationEngineParameters.LinearVelDisable &&
                simulationObj.AngularVelocity.Length() <= SimulationEngineParameters.AngularVelDisable)
            {
                if (simulationObj.SleepingFrameCount < SimulationEngineParameters.SleepingFrameLimit)
                    simulationObj.SetSleepingFrameCount(simulationObj.SleepingFrameCount + 1);
                else if (simulationObj.SleepingFrameCount >= SimulationEngineParameters.SleepingFrameLimit)
                {
                    simulationObj.SetLinearVelocity(new Vector3());
                    simulationObj.SetAngularVelocity(new Vector3());
                }
            }
            else
                simulationObj.SetSleepingFrameCount(0);
        }

        #endregion

        #region Position Based Integration

        private void UpdatePositionBasedVelocity(
            JacobianContact[] contact,
            SimulationObject[] simulationObj,
            SolutionValues[] X)
        {
            for (int i = 0; i < contact.Length; i++)
            {
                double impulse = X[i].X;

                JacobianContact ct = contact[i];

                SetPositionBasedVelocity(
                    simulationObj,
                    ct.LinearComponentA,
                    ct.AngularComponentA,
                    impulse,
                    ct.ObjectA);

                SetPositionBasedVelocity(
                    simulationObj,
                    ct.LinearComponentB,
                    ct.AngularComponentB,
                    impulse,
                    ct.ObjectB);
            }
        }

        private void SetPositionBasedVelocity(
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

                Vector3 linearVelocity = simObj.TempLinearVelocity +
                                         linearImpulse *
                                         simObj.InverseMass;

                Vector3 angularVelocity = simObj.TempAngularVelocity +
                                          simObj.InertiaTensor *
                                          angularImpuse;

                simulationObj[index].SetTempLinearVelocity(linearVelocity);
                simulationObj[index].SetTempAngularVelocity(angularVelocity);
            }
        }

        private void UpdateObjectPosition(
            SimulationObject[] simulationObj)
        {
            int index = 0;
            foreach (SimulationObject simObj in simulationObj)
            {
                if (simObj.ObjectType != ObjectType.StaticRigidBody)
                {
                    #region Linear Velocity

                    double linearVelocity = simObj.TempLinearVelocity.Length();

                    simObj.SetPosition(
                        simObj.Position +
                        TimeStep *
                        simObj.TempLinearVelocity);

                    simObj.SetTempLinearVelocity(new Vector3());

                    #endregion

                    #region Angular Velocity

                    double angularVelocity = simObj.TempAngularVelocity.Length();

                    Vector3 versor = simObj.TempAngularVelocity.Normalize();

                    double rotationAngle = angularVelocity * TimeStep;

                    var rotationQuaternion = new Quaternion(versor, rotationAngle);

                    simObj.SetRotationStatus(
                        (rotationQuaternion * simObj.RotationStatus).Normalize());

                    simObj.SetRotationMatrix(simObj.RotationStatus.ConvertToMatrix());

                    simObj.SetInertiaTensor(
                        (simObj.RotationMatrix * simObj.BaseInertiaTensor) *
                        simObj.RotationMatrix.Transpose());

                    simObj.SetTempAngularVelocity(new Vector3());

                    #endregion

                    #region Update AABB

                    if (simObj.ObjectGeometry != null &&
                        (linearVelocity > 0.0 || angularVelocity > 0.0))
                    {
                        simObj.ObjectGeometry.SetAABB(Helper.UpdateAABB(simObj));

                        simObj.SetTempLinearVelocity(new Vector3());
                        simObj.SetTempAngularVelocity(new Vector3());
                    }

                    #endregion
                }
                simulationObjects[index] = simObj;
                index++;
            }
        }

        #endregion

        #region IDisposable

        bool disposed = false;

		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}

		protected virtual void Dispose(bool disposing)
		{
			if (disposed)
				return;

			if (disposing)
			{
				simulationObjects = null;
				simulationJoints = null;
				collisionPoints = null;
				collisionPartitionedPoints = null;
				partitionedJoint = null;
				collisionEngine = null;
				contactPartitioningEngine = null;
				solver = null;
			}

			// Free any unmanaged objects here.
			//
			disposed = true;
		}

		#endregion

		#endregion

	}
}

