using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ContactPartitioning;

namespace SharpPhysicsEngine
{
	public sealed class SharpEngine: IDisposable
	{
		#region Public Properties

		/// <summary>
		/// The simulation parameters.
		/// </summary>
		public PhysicsEngineParameters EngineParameters { get; private set; }

		/// <summary>
		/// Gets the collision engine parameters.
		/// </summary>
		/// <value>The collision engine parameter.</value>
		public CollisionEngineParameters CollisionEngineParam { get; private set; }

		/// <summary>
		/// Gets the solver parameters.
		/// </summary>
		/// <value>The solver parameter.</value>
		public SolverParameters SolverParameters { get; private set; }

		/// <summary>
		/// Gets the time step.
		/// </summary>
		/// <value>The time step.</value>
		public double TimeStep { get; private set; }

		#endregion

		#region Private Properties

		/// <summary>
		/// The simulation Shapes.
		/// </summary>
		IShape[] Shapes;

		/// <summary>
		/// Subset of SoftShapes
		/// </summary>
		ISoftShape[] SoftShapes;

		/// <summary>
		/// The simulation joints.
		/// </summary>
		List<IConstraint> Joints;

		/// <summary>
		/// The collision engine.
		/// </summary>
		ICollisionEngine CollisionEngine;

		/// <summary>
		/// The collision points.
		/// </summary>
		CollisionPointStructure[] collisionPoints;

        /// <summary>
        /// Partitions elements
        /// </summary>
        List<Partition> Partitions;

		/// <summary>
		/// The solver.
		/// </summary>
		ISolver Solver;

		/// <summary>
		/// The contact partitioning engine.
		/// </summary>
		IContactPartitioningEngine contactPartitioningEngine;

		/// <summary>
		/// The last execution solver error.
		/// </summary>
		double solverError;

		/// <summary>
		/// Generate ID for shape of simulation
		/// </summary>
		HashGenerator HsGenerator;

		#endregion

		#region Constructor

		public SharpEngine (
			PhysicsEngineParameters simulationParameters,
			CollisionEngineParameters collisionEngineParameters,
			SolverParameters solverParameters)
		{
			SolverParameters = solverParameters;

			SetSolver(SolverType.NonLinearConjugateGradient);

			CollisionEngineParam = collisionEngineParameters;

			EngineParameters = simulationParameters;

			CollisionEngine = new CollisionDetectionEngine(collisionEngineParameters, EngineParameters.CollisionDistance);

			contactPartitioningEngine = new ContactPartitioningEngine();

			Shapes = new IShape[0];
			Joints = new List<IConstraint> ();
			HsGenerator = new HashGenerator();
		}

		public SharpEngine()
			: this(new PhysicsEngineParameters(), new CollisionEngineParameters(), new SolverParameters())
		{ }

		#endregion

		#region Public Methods

		#region Simulation Object 

		public void AddShape(IShape simulationObject)
		{
			ISoftShape softShape = simulationObject as ISoftShape;

			if (softShape != null)
			{
				((Identity)softShape).ID = HsGenerator.GetHash();
				foreach (var point in softShape.ShapePoints)
					((Identity)point).ID = HsGenerator.GetHash();
			}
			else
				((Identity)simulationObject).ID = HsGenerator.GetHash();
			
			if (Shapes != null && 
				Shapes.Length > 0) 
			{
				var bufferList = Shapes.ToList();
				bufferList.Add (simulationObject);
				Shapes = bufferList.ToArray ();
			} 
			else 
			{
				var bufferList = new List<IShape>();
				bufferList.Add (simulationObject);
				Shapes = bufferList.ToArray ();
			}

			SoftShapes = Shapes.Where(x => (x as ISoftShape) != null).Cast<ISoftShape>().ToArray();
		}

		public void RemoveShape(int shapeID)
		{
			if (Shapes != null) 
			{
				List<IShape> bufferList = Shapes.ToList();
				int shapeIndex = bufferList.FindIndex(x => x.GetID() == shapeID);

				if (shapeIndex >= 0)
				{
					#region Remove shape Constraint

					if (Joints != null)
					{
						for (int i = Joints.Count - 1; i >= 0; i--)
						{
							if (Joints[i].GetObjectIndexA() == shapeID ||
								Joints[i].GetObjectIndexB() == shapeID)
							{
								RemoveJoint(i);
							}
						}
					}

					#endregion

					#region Remove shape

					bufferList.RemoveAt(shapeIndex);
					Shapes = bufferList.ToArray();

					#endregion
				}
			}
		}

		public void RemoveShapes()
		{
			Shapes = new IShape[0];
			Joints = new List<IConstraint>();
		}

		public IShape GetShape(int shapeID)
		{
			return Shapes.FirstOrDefault(x => x.GetID() == shapeID);
		}

		public int ShapesCount()
		{
			return Shapes.Length;
		}

		public IShape[] GetShapes()
		{
			return Shapes;
		}
					
		#endregion

		#region Simulation Joint

		public void AddJoint(IConstraint joint)
		{
			if (Joints != null &&
				Joints.Count > 0)
			{
				Joints.Add(joint);
			}
			else
			{
				Joints = new List<IConstraint>();
				Joints.Add(joint);
			}
		}

		public void RemoveJoint(int jointIndex)
		{
			if (Joints != null &&
			   Joints.Count > jointIndex)
			{
				Joints.RemoveAt(jointIndex);
			}
		}

		public void RemoveJoints()
		{
			Joints = new List<IConstraint>();
		}

		public IConstraint GetJoints(int constraintId)
		{
			if (Joints != null &&
				Joints.Count > constraintId)
				return Joints[constraintId];
			
			return null;
		}

		public int JointsCount()
		{
			return Joints.Count;
		}

		public List<IConstraint> GetJoints()
		{
			return new List<IConstraint>(Joints);
		}

		#endregion

		#region Collision Engine

		public List<CollisionPointStructure> GetCollisionPointStrucureList()
		{
			if (collisionPoints == null)
				return new List<CollisionPointStructure>();
			
			return new List<CollisionPointStructure>(collisionPoints);
		}

		public List<Partition> GetPartitionedCollisionPoints()
		{
			return Partitions;
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
					Solver = new ProjectedGaussSeidel(SolverParameters);
					break;

				case SolverType.NonLinearConjugateGradient:
					Solver = new NonLinearConjugateGradient(SolverParameters);
					break;

				case SolverType.ConjugateGradient:
					Solver = new ConjugateGradient(SolverParameters);
					break;
																						 
				default:
					Solver = new ProjectedGaussSeidel(SolverParameters);
					break;
			}
		}

		#endregion

		#region Engine Execution

		/// <summary>
		/// Runs the engine.
		/// </summary>
		public void Simulate(double? timeStep = null)
		{
			TimeStep = (timeStep.HasValue) ?
					TimeStep = timeStep.Value :
					EngineParameters.TimeStep;

			#region Simulation Workflow

			CollisionDetection();

			PartitionEngineExecute();

			PhysicsExecutionFlow();

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

		#region Position Correction

		private bool PhysicsJointPositionCorrection()
		{
			bool positionUpdated = false;

			if (Joints.Count > 0)
			{
				if (Partitions != null)
				{
					double baumgarteStabilizationValue = 1.0 / TimeStep;

					for (int i = 0; i < Partitions.Count; i++)
					{
						if (EngineParameters.PositionBasedJointIterations > 0)
						{
							JacobianConstraint[] jointConstraints = GetJacobianJointConstraint(
																	   Partitions[i].PartitionedJoints,
																	   Shapes,
																	   baumgarteStabilizationValue).ToArray();

							LinearProblemProperties collisionErrorLCP = BuildLCPMatrix(
								jointConstraints,
								EngineParameters.PositionStabilization);

							if (collisionErrorLCP != null)
							{
								Solver.GetSolverParameters().SetSolverMaxIteration(EngineParameters.PositionBasedJointIterations);

								SolutionValues[] correctionValues = Solver.Solve(collisionErrorLCP);

								UpdatePositionBasedVelocity(jointConstraints, correctionValues);

								positionUpdated = true;
							}
						}
					}
					#region Position and Velocity integration

					if (positionUpdated)
						UpdateObjectPosition(Shapes);

					#endregion
				}
			}

			return positionUpdated;
		}

		#endregion

		private JacobianConstraint[] ContactSorting(JacobianConstraint[] jacobianContact)
		{
			var sorted = jacobianContact.Select((x, i) => new KeyValuePair<JacobianConstraint, int>(x, i)).
				OrderBy(x => Math.Abs(x.Key.B)).ToArray();

			int[] sortedIndex = sorted.Select(x => x.Value).ToArray();
			JacobianConstraint[] sortedContact = sorted.Select(x => x.Key).ToArray();

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

			if (EngineParameters.PositionStabilization)
			{
				bool positionUpdated = PhysicsJointPositionCorrection();

				if (positionUpdated)
				{
					CollisionDetection();
					PartitionEngineExecute();
				}
			}
			
			if (Partitions != null) 
			{
				bool convertSetting = false;
				if (EngineParameters.PositionStabilization)
				{
					EngineParameters.SetPositionStabilization(false);
					convertSetting = true;
				}

				for (int i = 0; i < Partitions.Count;i++)
				{
					JacobianConstraint[] jacobianConstraints = GetJacobianConstraints(
																   Partitions[i].PartitionedCollisionPoints.ToArray(),
                                                                   Partitions[i].PartitionedJoints,
																   Shapes,
																   EngineParameters).ToArray();

					if (jacobianConstraints.Length > 0)
					{
						SolutionValues[] overallSolution = new SolutionValues[jacobianConstraints.Length];

						#region Solve Normal And Friction Constraints

						if (EngineParameters.FrictionAndNormalIterations > 0)
						{
							JacobianConstraint[] frictionConstraint = Helper.FilterConstraints(jacobianConstraints,
																					   ConstraintType.Friction,
																					   ConstraintType.Collision);

							LinearProblemProperties frictionLCP = BuildLCPMatrix(
																	frictionConstraint,
																	EngineParameters.PositionStabilization);

							SolutionValues[] contactSolution = BuildMatrixAndExecuteSolver(
														frictionConstraint,
														frictionLCP,
														EngineParameters.FrictionAndNormalIterations);
						}

						#endregion

						#region Solve Joint Constraint

						if (Joints.Count > 0 &&
							EngineParameters.JointsIterations > 0)
						{
							JacobianConstraint[] jointConstraints = Helper.FindJointConstraints(jacobianConstraints);

							LinearProblemProperties jointLCP = BuildLCPMatrix(
																	jointConstraints,
																	EngineParameters.PositionStabilization);

							SolutionValues[] jointSolution = BuildMatrixAndExecuteSolver(
														jointConstraints,
														jointLCP,
														EngineParameters.JointsIterations);
						}

						#endregion

						#region Solve Overall Constraints

						//jacobianConstraints = ContactSorting(jacobianConstraints);

						LinearProblemProperties overallLCP = BuildLCPMatrix(
																jacobianConstraints,
																EngineParameters.PositionStabilization);

						if (overallLCP != null &&
						   EngineParameters.OverallConstraintsIterations > 0)
						{
							Solver.GetSolverParameters().SetSolverMaxIteration(EngineParameters.OverallConstraintsIterations);

							overallSolution = Solver.Solve(overallLCP);

							double[] overallError = new double[overallLCP.Count];

							//SolverParameters test = new SolverParameters(300, solver.GetSolverParameters().ErrorTolerance, solver.GetSolverParameters().SOR, solver.GetSolverParameters().MaxThreadNumber, solver.GetSolverParameters().SORStep, solver.GetSolverParameters().DynamicSORUpdate);

							//ProjectedGaussSeidel testVerifica = new ProjectedGaussSeidel(test);

							//SolutionValues[] sol = testVerifica.Solve(overallLCP);

							Console.WriteLine("error " + SolverHelper.ComputeSolverError(overallLCP, overallSolution));
							//Console.WriteLine("errorTest " + SolverHelper.ComputeSolverError(overallLCP, sol));
						}
						else if (EngineParameters.OverallConstraintsIterations == 0)
						{
							for (int j = 0; j < overallSolution.Length; j++)
								overallSolution[j].X = jacobianConstraints[j].StartImpulse.StartImpulseValue;
						}

						UpdateVelocity(jacobianConstraints, overallSolution);

						#endregion
					}
				}
				if (convertSetting)
				{
					EngineParameters.SetPositionStabilization(true);
				}
			}

			#endregion

			#region Position and Velocity integration

			IntegrateObjectsPosition (Shapes);

			#endregion

			stopwatch.Stop();

			Console.WriteLine("Inner Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

		}

		#region Collision Detection

		/// <summary>
		/// Collisions detection.
		/// </summary>
		private void CollisionDetection()
		{
			#region Init WarmStarting

			List<CollisionPointStructure> collisionPointsBuffer = null;

			if (collisionPoints != null &&
				collisionPoints.Length > 0)
				collisionPointsBuffer = new List<CollisionPointStructure>(collisionPoints);

			#endregion

			#region Find New Collision Points

			//Creo l'array contenente la geometria degli oggetti
			IShape[] simShapes = Array.ConvertAll (
							Shapes, 
							item => (item.ExcludeFromCollisionDetection) ? null : item);

			var stopwatch = new Stopwatch();

			stopwatch.Reset ();
			stopwatch.Start ();

			//Eseguo il motore che gestisce le collisioni
			collisionPoints = CollisionEngine.Execute(simShapes).ToArray();

			#endregion

			#region WarmStarting

			//if (collisionPointsBuffer != null &&
			//    collisionPointsBuffer.Count > 0)
			//    WarmStarting(collisionPointsBuffer);

			#endregion

			stopwatch.Stop ();

			Console.WriteLine("Collision Elapsed={0}",stopwatch.ElapsedMilliseconds);
		}
        
        #endregion

		#region Contact Partitioning

		private void PartitionEngineExecute()
		{
            Partitions = null;
                                    						
			List<SpatialPartition> spatialPartitions = contactPartitioningEngine.CalculateSpatialPartitioning(
													collisionPoints,
													Joints,
													Shapes);

			if (spatialPartitions != null)
			{
                Partitions = new List<Partition>();

				for (int i = 0; i < spatialPartitions.Count; i++)
				{
                    Partition partitionItem = new Partition();
                    					
					for (int j = 0; j < spatialPartitions[i].ObjectList.Count; j++)
					{
						if (spatialPartitions[i].ObjectList[j].Type == ContactGroupType.Collision)
						{
							CollisionPointStructure cpStruct = Helper.Find(
								collisionPoints,
								spatialPartitions[i].ObjectList[j]);

                            if (cpStruct != null)
                                partitionItem.PartitionedCollisionPoints.Add(cpStruct);
                            
						}
						else
						{
							IConstraint smJoint = Joints.Find(item =>
												  item.GetObjectIndexA() == spatialPartitions[i].ObjectList[j].IndexA &&
												  item.GetObjectIndexB() == spatialPartitions[i].ObjectList[j].IndexB &&
												  item.GetKeyIndex() == spatialPartitions[i].ObjectList[j].KeyIndex);

                            partitionItem.PartitionedJoints.Add(smJoint);
                            
						}
					}

                    ////Add Soft Body Constraints


                    Partitions.Add(partitionItem);
				}
			}
            else if(SoftShapes.Length > 0)
            {
                Partitions = new List<Partition>();

                foreach (var softShape in SoftShapes)
                {
                    Partition partitionItem = new Partition();

                    partitionItem.PartitionedJoints.AddRange(softShape.SoftConstraint);
                    Partitions.Add(partitionItem);
                }
            }
		}

		#endregion

		#region Jacobian Constraint

		public List<JacobianConstraint> GetJacobianConstraints(
			CollisionPointStructure[] collisionPointsStruct,
			List<IConstraint> simulationJointList,
			IShape[] simulationObjs,
			PhysicsEngineParameters simulationParameters)
		{
			var constraint = new List<JacobianConstraint>();

			#region Collision Contact

			constraint.AddRange(
				ContactConstraint.BuildJoints(
					collisionPointsStruct,
					simulationObjs,
					simulationParameters));

			#endregion

			#region Joint

			foreach (IConstraintBuilder constraintItem in simulationJointList)
				constraint.AddRange(constraintItem.BuildJacobian());

			#endregion

			return constraint;
		}

        public List<JacobianConstraint> GetSoftBodyConstraints()
        {
            var constraints = new List<JacobianConstraint>();

            foreach (ISoftShape softShape in SoftShapes)
            {
                foreach (var item in softShape.SoftConstraint)
                    constraints.AddRange(item.BuildJacobian());
            }

            return constraints;
        }

        public List<JacobianConstraint> GetJacobianJointConstraint(
			List<IConstraint> simulationJointList,
			IShape[] simulationObjs,
			double? stabilizationCoeff = null)
		{
			var constraint = new List<JacobianConstraint>();
						
			if (stabilizationCoeff.HasValue)
			{
					foreach (IConstraintBuilder constraintItem in simulationJointList)
					constraint.AddRange(constraintItem.BuildJacobian(stabilizationCoeff));
			}
			else
			{
				foreach (IConstraintBuilder constraintItem in simulationJointList)
					constraint.AddRange(constraintItem.BuildJacobian());
			}

			return constraint;
		}

		#endregion

		#region Solver Matrix Builder

		private SolutionValues[] BuildMatrixAndExecuteSolver(
			JacobianConstraint[] contactConstraints,
			LinearProblemProperties linearProblemProperties,
			int nIterations)
		{
			if (linearProblemProperties != null)
			{
				Solver.GetSolverParameters().SetSolverMaxIteration(nIterations);

				SolutionValues[]  solutionValues = Solver.Solve(linearProblemProperties);

				for (int j = 0; j < contactConstraints.Length; j++)
				{
					contactConstraints[j].StartImpulse.SetStartValue(solutionValues[j].X);
				}

				return solutionValues;
			}

			return null;
		}

        private struct HashSetHelper: IEquatable<HashSetHelper>
        {
            private readonly int index, objectID;

            public int Index { get { return index; } }
            public int ObjectID { get { return objectID; } }

            public HashSetHelper(int index, int objectID)
            {
                this.index = index;
                this.objectID = objectID;
            }

            public override int GetHashCode()
            {
                return 31 * index + 17 * objectID; // Or something like that
            }

            public override bool Equals(object obj)
            {
                return obj is HashSetHelper && Equals((HashSetHelper)obj);
            }

            public bool Equals(HashSetHelper p)
            {
                return index == p.Index && objectID == p.ObjectID;
            }

        }

        /// <summary>
        /// Builds the LCP matrix for solver.
        /// </summary>
        private LinearProblemProperties BuildLCPMatrix(
			JacobianConstraint[] constraint,
			bool positionStabilization = false)
		{
			if (constraint.Length > 0) 
			{				
				double[] B = new double[constraint.Length];
				double[] D = new double[constraint.Length];
				ConstraintType[] constraintsType = new ConstraintType[constraint.Length];
				double[] constraintsLimit = new double[constraint.Length];
				List<int?>[] constraints = new List<int?>[constraint.Length];
				
				List<int>[] index = new List<int>[constraint.Length];
				List<double>[] value = new List<double>[constraint.Length];
                HashSet<HashSetHelper> indexA = new HashSet<HashSetHelper>();
                HashSet<HashSetHelper> indexB = new HashSet<HashSetHelper>();

				for (int i = 0; i < constraint.Length; i++) 
				{
					index [i] = new List<int> ();
					value [i] = new List<double> ();
					constraints[i] = new List<int?>();

                    indexA.Add(new HashSetHelper(i, constraint[i].ObjectA.GetID()));
                    indexB.Add(new HashSetHelper(i, constraint[i].ObjectB.GetID()));
                }

				//Critical section variable
				var sync = new object ();

				Parallel.For(0,
					constraint.Length,
					new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
					i =>
					{
						JacobianConstraint contactA = constraint[i];

						if (positionStabilization)
							B[i] = contactA.CorrectionValue;
						else
							B[i] = -(contactA.B - ((contactA.CorrectionValue) < 0 ? Math.Max(contactA.CorrectionValue, -EngineParameters.MaxCorrectionValue) :
																					Math.Min(contactA.CorrectionValue, EngineParameters.MaxCorrectionValue)));

						if (contactA.ContactReference.HasValue)
							constraints[i].Add(contactA.ContactReference);

						constraintsLimit[i] = contactA.ConstraintLimit;
						constraintsType[i] = contactA.Type;

						double mValue = GetLCPDiagonalValue(contactA, contactA);

						//Diagonal value
						mValue += contactA.CFM +
								  EngineParameters.CFM +
								  1E-40;

						D[i] = 1.0 / mValue;

                        int contactA_ID_A = contactA.ObjectA.GetID();
                        int contactA_ID_B = contactA.ObjectB.GetID();

                        List<int> testIndexAA_BA = new List<int>();
                        List<int> testIndexAA_BB = new List<int>();
                        List<int> testIndexAB_BA = new List<int>();
                        List<int> testIndexAB_BB = new List<int>();

                        for (int j = i + 1; j < constraint.Length; j++)
						{
                            JacobianConstraint contactB = constraint[j];

                            int contactB_ID_A = contactB.ObjectA.GetID();
                            int contactB_ID_B = contactB.ObjectA.GetID();

                            //indexA.Contains(new HashSetHelper(j, contactB_ID_A));

							

                            mValue = 0.0;

                            if (contactA_ID_A == contactB_ID_A)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentA,
                                    contactB.LinearComponentA,
                                    contactA.AngularComponentA,
                                    contactB.AngularComponentA,
                                    contactA.ObjectA.InverseMass,
                                    contactA.ObjectA.InertiaTensor);                                
                            }
                            else if (contactA_ID_A == contactB_ID_B)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentA,
                                    contactB.LinearComponentB,
                                    contactA.AngularComponentA,
                                    contactB.AngularComponentB,
                                    contactA.ObjectA.InverseMass,
                                    contactA.ObjectA.InertiaTensor);
                            }

                            if (contactA_ID_B == contactB_ID_A)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentB,
                                    contactB.LinearComponentA,
                                    contactA.AngularComponentB,
                                    contactB.AngularComponentA,
                                    contactA.ObjectB.InverseMass,
                                    contactA.ObjectB.InertiaTensor);
                            }
                            else if (contactA_ID_B == contactB_ID_B)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentB,
                                    contactB.LinearComponentB,
                                    contactA.AngularComponentB,
                                    contactB.AngularComponentB,
                                    contactA.ObjectB.InverseMass,
                                    contactA.ObjectB.InertiaTensor);
                            }
                                                        
                            if (Math.Abs(mValue) > 1E-32)
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
                    });

				SparseElement[] M = new SparseElement[constraint.Length];
				int?[][] constraintsArray = new int?[constraint.Length][];
				for (int i = 0; i < constraint.Length; i++) 
				{
					M [i] = new SparseElement (
						value [i].ToArray (),
						index [i].ToArray (),
						constraint.Length);

					constraintsArray[i] = constraints[i].ToArray();
				}
				
				return new LinearProblemProperties (
					M,
					B,
					D,
					constraintsLimit,
					constraintsType,
					constraintsArray);
			}

			return null;
		}

        private double GetLCPMatrixValue(
            Vector3 linearComponentA,
            Vector3 linearComponentB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            double inverseMass,
            Matrix3x3 inertiaTensor)
        {
            double linear = linearComponentA.Dot(
                        linearComponentB * inverseMass);

            double angular = angularComponentA.Dot(
                        inertiaTensor * angularComponentB);

            return linear + angular;
        }

        
        private double GetLCPDiagonalValue(
            JacobianConstraint contactA,
            JacobianConstraint contactB)
        {
            double linearA = contactA.LinearComponentA.Dot(
                    contactB.LinearComponentA * contactA.ObjectA.InverseMass);

            double angularA = contactA.AngularComponentA.Dot(
                    contactA.ObjectA.InertiaTensor * contactB.AngularComponentA);

            double linearB = contactA.LinearComponentB.Dot(
                    contactB.LinearComponentB * contactA.ObjectB.InverseMass);

            double angularB = contactA.AngularComponentB.Dot(
                    contactA.ObjectB.InertiaTensor * contactB.AngularComponentB);
            
            return (linearA + angularA) +
                   (linearB + angularB);
        }

		#endregion

		#region Integrate Velocity and Position

		/// <summary>
		/// Updates velocity of the simulations objects.
		/// </summary>
		private void UpdateVelocity(
			JacobianConstraint[] contact,
			SolutionValues[] X)
		{
			for (int i = 0; i < contact.Length; i++)
			{
				if (Math.Abs(X[i].X) > 1E-50)
				{
					double impulse = X[i].X;

					JacobianConstraint ct = contact[i];

					UpdateObjectVelocity(
						ct.ObjectA,
						ct.LinearComponentA,
						ct.AngularComponentA,
						impulse);

					UpdateObjectVelocity(
						ct.ObjectB,
						ct.LinearComponentB,
						ct.AngularComponentB,
						impulse);

					ct.StartImpulse.SetStartValue(impulse * EngineParameters.WarmStartingValue);
				}
			}
		}

		/// <summary>
		/// Updates the object velocity.
		/// </summary>
		private void UpdateObjectVelocity(
			IShapeCommon simObj,
			Vector3 linearComponent,
			Vector3 angularComponent,
			double X)
		{
				if (simObj.ObjectType != ObjectType.StaticBody) 
			{
				Vector3 linearImpulse = X * linearComponent;
				Vector3 angularImpuse = X * angularComponent;

				Vector3 linearVelocity = simObj.LinearVelocity +
										 linearImpulse * 
										 simObj.InverseMass;

				Vector3 angularVelocity = simObj.AngularVelocity +
										  (simObj.InertiaTensor *
										  angularImpuse);

                //TODO implementare x Soft Body
				simObj.SetLinearVelocity (linearVelocity);
				simObj.SetAngularVelocity (angularVelocity);
			}
		}

		/// <summary>
		/// Integrates the objects position for ConvexShape and CompoundShape.
		/// </summary>
		private void IntegrateObjectsPosition(IShape[] shapes)
		{
			foreach (var shape in shapes) 
			{
					if (shape.ObjectType != ObjectType.StaticBody) 
				{
					ISoftShape softShape = shape as ISoftShape;

					if (softShape == null)
						IntegrateRigidShapePosition(shape);
					else
						IntegrateSoftShapePosition(softShape);                    
				}
			}
		}

		private void IntegrateRigidShapePosition(IShape shape)
		{
			#region Linear Velocity

			shape.SetPosition(
					shape.Position +
					TimeStep *
					shape.LinearVelocity);

			shape.SetLinearVelocity(shape.LinearVelocity +
				(shape.ForceValue * shape.InverseMass) +
				(TimeStep * EngineParameters.ExternalForce));

			shape.SetForce(new Vector3());

			double linearVelocity = shape.LinearVelocity.Length();

			#endregion

			#region Angular Velocity

			double angularVelocity = shape.AngularVelocity.Length();

			Vector3 versor = shape.AngularVelocity.Normalize();

			double rotationAngle = angularVelocity * TimeStep;

			var rotationQuaternion = new Quaternion(versor, rotationAngle);

			shape.SetRotationStatus((rotationQuaternion * shape.RotationStatus).Normalize());

			shape.SetRotationMatrix(shape.RotationStatus.ConvertToMatrix());

			shape.SetInertiaTensor(
				(shape.RotationMatrix * shape.BaseInertiaTensor) *
				shape.RotationMatrix.Transpose());

			shape.SetAngularVelocity(shape.AngularVelocity +
									  shape.InertiaTensor *
									  shape.TorqueValue);

			angularVelocity = shape.AngularVelocity.Length();
			shape.SetTorque(new Vector3());

			#endregion

			#region Sleeping Object

			if (EngineParameters.SleepingObject)
				ObjectSleep(shape);

			#endregion

			#region Update AABB

			if (ShapeDefinition.Helper.GetGeometry(shape) != null &&
				(linearVelocity > 0.0 || angularVelocity > 0.0))
			{
				shape.SetAABB();
			}

			#endregion
		}

		private void IntegrateSoftShapePosition(ISoftShape shape)
		{
			foreach(var point in shape.ShapePoints)
			{
				#region Linear Velocity

				point.SetPosition(
						point.Position +
						TimeStep *
						point.LinearVelocity);

				point.SetLinearVelocity(point.LinearVelocity +
					(point.ForceValue * point.InverseMass) +
					(TimeStep * EngineParameters.ExternalForce));

				point.SetForce(new Vector3());

				double linearVelocity = point.LinearVelocity.Length();

				#endregion
			}
						
			shape.SetAABB();
		}

		private void ObjectSleep(IShape simulationObj)
		{
			if (simulationObj.LinearVelocity.Length() <= EngineParameters.LinearVelDisable &&
				simulationObj.AngularVelocity.Length() <= EngineParameters.AngularVelDisable)
			{
				if (simulationObj.SleepingFrameCount < EngineParameters.SleepingFrameLimit)
					simulationObj.SetSleepingFrameCount(simulationObj.SleepingFrameCount + 1);
				else if (simulationObj.SleepingFrameCount >= EngineParameters.SleepingFrameLimit)
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
			JacobianConstraint[] contact,
			SolutionValues[] X)
		{
			for (int i = 0; i < contact.Length; i++)
			{
				double impulse = X[i].X;

				JacobianConstraint ct = contact[i];

				SetPositionBasedVelocity(
					ct.ObjectA,
					ct.LinearComponentA,
					ct.AngularComponentA,
					impulse);

				SetPositionBasedVelocity(
					ct.ObjectB,
					ct.LinearComponentB,
					ct.AngularComponentB,
					impulse);
			}
		}

		private void SetPositionBasedVelocity(
			IShapeCommon simObj,
			Vector3 linearComponent,
			Vector3 angularComponent,
			double X)
		{
			if (simObj.ObjectType != ObjectType.StaticBody)
			{
				Vector3 linearImpulse = X * linearComponent;
				Vector3 angularImpuse = X * angularComponent;

				Vector3 linearVelocity = simObj.TempLinearVelocity +
										 linearImpulse *
										 simObj.InverseMass;

				Vector3 angularVelocity = simObj.TempAngularVelocity +
										  simObj.InertiaTensor *
										  angularImpuse;

				simObj.SetTempLinearVelocity(linearVelocity);
				simObj.SetTempAngularVelocity(angularVelocity);
			}
		}

		private void UpdateObjectPosition(
			IShape[] simulationObj)
		{
			int index = 0;
			foreach (IShape simObj in simulationObj)
			{
				if (simObj.ObjectType != ObjectType.StaticBody)
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

					if (ShapeDefinition.Helper.GetGeometry(simObj) != null &&
						(linearVelocity > 0.0 || angularVelocity > 0.0))
					{
						simObj.SetAABB();

						simObj.SetTempLinearVelocity(new Vector3());
						simObj.SetTempAngularVelocity(new Vector3());
					}

					#endregion
				}
				Shapes[index] = simObj;
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

		private void Dispose(bool disposing)
		{
			if (disposed)
				return;

			if (disposing)
			{
				Shapes = null;
				Joints = null;
				collisionPoints = null;
				Partitions = null;
				CollisionEngine = null;
				contactPartitioningEngine = null;
				Solver = null;
			}

			// Free any unmanaged objects here.
			//
			disposed = true;
		}

		#endregion

		#endregion

	}
}

