/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

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
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.Wrapper;
using SharpPhysicsEngine.Wrapper.Joint;
using System.Collections.Concurrent;

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
        /// User Collision Shapes
        /// </summary>
        private List<ICollisionShape> CollisionShapes;

        /// <summary>
        /// User Collision Joints
        /// </summary>
        private List<ICollisionJoint> CollisionJoints;
        
		/// <summary>
		/// The simulation Shapes.
		/// </summary>
		private IShape[] Shapes;

		/// <summary>
		/// Subset of SoftShapes
		/// </summary>
		private ISoftShape[] SoftShapes;

		/// <summary>
		/// The simulation joints.
		/// </summary>
		private List<IConstraint> Joints;

		/// <summary>
		/// The collision engine.
		/// </summary>
		private ICollisionEngine CollisionEngine;

		/// <summary>
		/// The collision points.
		/// </summary>
		private CollisionPointStructure[] collisionPoints;

		/// <summary>
		/// Partitions elements
		/// </summary>
		private List<Partition> Partitions;

		/// <summary>
		/// The solver.
		/// </summary>
		private ISolver Solver;

		/// <summary>
		/// The contact partitioning engine.
		/// </summary>
		private readonly IContactPartitioningEngine contactPartitioningEngine;

		/// <summary>
		/// The last execution solver error.
		/// </summary>
		double solverError;

		/// <summary>
		/// Generate ID for shape of simulation
		/// </summary>
		private readonly HashGenerator HsGenerator;

		private readonly LinearProblemBuilder linearProblemBuilder;

		private readonly IntegrationHelper integrationHelper;

		private readonly ContactConstraintBuilder contactConstraintBuilder;

		#endregion

		#region Constructor

		public SharpEngine (
			PhysicsEngineParameters simulationParameters,
			CollisionEngineParameters collisionEngineParameters,
			SolverParameters solverParameters)
		{
			SolverParameters = solverParameters;

			SetSolver(SolverType.ProjectedGaussSeidel);

			CollisionEngineParam = collisionEngineParameters;

			EngineParameters = simulationParameters;

			CollisionEngine = new CollisionDetectionEngine(collisionEngineParameters, EngineParameters.CollisionDistance);

			contactPartitioningEngine = new ContactPartitioningEngine();

			Shapes = new IShape[0];
            CollisionShapes = new List<ICollisionShape>();
            CollisionJoints = new List<ICollisionJoint>();
			Joints = new List<IConstraint> ();
			HsGenerator = new HashGenerator();
			linearProblemBuilder = new LinearProblemBuilder(EngineParameters);
			integrationHelper = new IntegrationHelper(EngineParameters);
			contactConstraintBuilder = new ContactConstraintBuilder(EngineParameters);

			//int minWorker, minIOC;
			//// Get the current settings.
			//ThreadPool.GetMinThreads(out minWorker, out minIOC);
			//// Change the minimum number of worker threads to four, but
			//// keep the old setting for minimum asynchronous I/O 
			//// completion threads.
			//ThreadPool.SetMinThreads(4, minIOC);
		}

		public SharpEngine()
			: this(new PhysicsEngineParameters(), new CollisionEngineParameters(), new SolverParameters())
		{ }

		#endregion

		#region Public Methods

		#region Simulation Object Methods

		public void AddShape(ICollisionShape simulationObject)
		{
            CollisionShapes.Add(simulationObject);
            IShape simObj = ((IMapper)simulationObject).GetShape();

            if (simObj is ISoftShape softShape)
            {
                ((IDentity)softShape).SetID(HsGenerator.GetHash());
                foreach (var point in softShape.ShapePoints)
                    ((IDentity)point).SetID(HsGenerator.GetHash());
            }
            else
                ((IDentity)simObj).SetID(HsGenerator.GetHash());

            if (Shapes != null && 
				Shapes.Length > 0) 
			{
				var bufferList = Shapes.ToList();
				bufferList.Add (simObj);
				Shapes = bufferList.ToArray ();
			} 
			else 
			{
                var bufferList = new List<IShape>
                {
                    simObj
                };
                Shapes = bufferList.ToArray ();
			}

			SoftShapes = Shapes.Where(x => (x as ISoftShape) != null).Cast<ISoftShape>().ToArray();
		}

		public void RemoveShape(int shapeID)
		{
			if (Shapes != null) 
			{
				List<IShape> bufferList = Shapes.ToList();
				int shapeIndex = bufferList.FindIndex(x => x.ID == shapeID);

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
            CollisionShapes.Clear();
			Joints = new List<IConstraint>();
		}

		public ICollisionShape GetShape(int shapeID)
		{
			return CollisionShapes.FirstOrDefault(x => x.GetID() == shapeID);
		}

		public int ShapesCount()
		{
			return Shapes.Length;
		}

		public ICollisionShape[] GetShapes()
		{
			return CollisionShapes.ToArray();
		}
					
		#endregion

		#region Simulation Joint

		public void AddJoint(ICollisionJoint joint)
		{
            CollisionJoints.Add(joint);

            var mappedJoint = ((IMapperJoint)joint).GetJoint();

			if (Joints != null &&
				Joints.Count > 0)
			{
				Joints.Add(mappedJoint);
			}
			else
			{
                Joints = new List<IConstraint>
                {
                    mappedJoint
                };
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

		public ICollisionJoint GetJoints(int jointIndex)
		{
            if (Joints != null &&
                jointIndex >= 0 &&
                jointIndex < CollisionJoints.Count)
                return CollisionJoints[jointIndex];
			
			return null;
		}

		public int JointsCount()
		{
			return Joints.Count;
		}

		public List<ICollisionJoint> GetJoints()
		{
			return CollisionJoints;
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
					Solver = new ProjectedGaussSeidel(SolverParameters);
					break;

				case SolverType.NonLinearConjugateGradient:
					Solver = new NonLinearConjugateGradient(SolverParameters);
					break;

				case SolverType.ProjectedConjugateGradient:
					Solver = new ProjectedConjugateGradient(SolverParameters);
					break;

                case SolverType.RedBlackProjectedGaussSeidel:
                    Solver = new RedBlackProjectedGaussSeidel(SolverParameters);
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
		public void Simulate(double timeStep)
		{
			TimeStep = timeStep;

			#region Simulation Workflow

			CollisionDetection();

			PartitionEngineExecute();

			PhysicsExecutionFlow();

			#endregion
		}

		public void Simulate()
		{
			Simulate(EngineParameters.TimeStep);
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

        private JacobianConstraint[] ContactSorting(JacobianConstraint[] jacobianContact)
		{
			var sorted = jacobianContact.Select((x, i) => new KeyValuePair<JacobianConstraint, int>(x, i)).
				OrderBy(x => x.Key.ObjectA.Position * EngineParameters.ExternalForce).ToArray();

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
            			
			if (Partitions != null) 
			{
				for (int i = 0; i < Partitions.Count;i++)
				{
                    JacobianConstraint[] jacobianConstraints = GetJacobianConstraints(
																            Partitions[i].PartitionedCollisionPoints.ToArray(),
																            Partitions[i].PartitionedJoints,
																            Shapes,
																            EngineParameters);

					if (jacobianConstraints.Length > 0)
					{
                        //Contact sorting
                        //JacobianConstraint[] jacobianConstraints1 = ContactSorting(jacobianConstraints);


                        double[] overallSolution = new double[jacobianConstraints.Length];
                        	                      
                        LinearProblemProperties overallLCP = linearProblemBuilder.BuildLCP(
                                                                jacobianConstraints);

                        //LinearProblemProperties overallLCP1 = linearProblemBuilder.BuildLCP(
                        //                                        jacobianConstraints1);

                        if (overallLCP != null &&
						   EngineParameters.OverallConstraintsIterations > 0)
						{
                            var stopwatch1 = new Stopwatch();

                            stopwatch1.Reset();

                            stopwatch1.Start();

                            Solver.GetSolverParameters().SetSolverMaxIteration(EngineParameters.OverallConstraintsIterations);

							overallSolution = Solver.Solve(overallLCP, new double[overallLCP.Count]);

                           //var overallSolution1 = Solver.Solve(overallLCP1, new double[overallLCP.Count]);

                            stopwatch1.Stop();

                            Console.WriteLine("Solver ={0}", stopwatch1.ElapsedMilliseconds);
                            
                        }
                        
                        integrationHelper.UpdateVelocity(jacobianConstraints, overallSolution);
					}
				}
			}

			#endregion

			#region Position and Velocity integration

			integrationHelper.IntegrateObjectsPosition(ref Shapes, TimeStep);

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
                    HashSet<int> objectIndex = new HashSet<int>();
										
					for (int j = 0; j < spatialPartitions[i].ObjectList.Count; j++)
					{
						if (spatialPartitions[i].ObjectList[j].Type == ContactGroupType.Collision)
						{
							CollisionPointStructure cpStruct = ConstraintHelper.Find(
								collisionPoints,
								spatialPartitions[i].ObjectList[j]);

							if (cpStruct != null)
								partitionItem.PartitionedCollisionPoints.Add(cpStruct);

                            objectIndex.Add(cpStruct.ObjectIndexA);
                            objectIndex.Add(cpStruct.ObjectIndexB);

                        }
						else
						{
							IConstraint smJoint = Joints.Find(item =>
												  item.GetObjectIndexA() == spatialPartitions[i].ObjectList[j].IndexA &&
												  item.GetObjectIndexB() == spatialPartitions[i].ObjectList[j].IndexB &&
												  item.GetKeyIndex() == spatialPartitions[i].ObjectList[j].KeyIndex);

							partitionItem.PartitionedJoints.Add(smJoint);

                            objectIndex.Add(smJoint.GetObjectIndexA());
                            objectIndex.Add(smJoint.GetObjectIndexB());

                        }
					}

                    ////Add Soft Body Constraints
                    foreach (var item in objectIndex)
                    {
                        if (Shapes[item] is ISoftShape softShape)
                            partitionItem.PartitionedJoints.AddRange(softShape.SoftConstraint);
                    }
                    
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

		private List<JacobianConstraint> GetContactJacobianConstraints(
            CollisionPointStructure[] collisionPointsStruct,
            IShape[] simulationObjs)
        {
            List<JacobianConstraint> jacobianConstraints = new List<JacobianConstraint>();

            var sync = new object();

            Parallel.ForEach(
                collisionPointsStruct, 
                new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                item =>
                {
                    IShape objectA = simulationObjs.First(x => x.ID == item.ObjectIndexA);
                    IShape objectB = simulationObjs.First(x => x.ID == item.ObjectIndexB);

                    List<JacobianConstraint> constraintsBuf = contactConstraintBuilder.BuildJoints(item, objectA, objectB);

                    lock (sync)
                    {
                        jacobianConstraints.AddRange(constraintsBuf);
                    }
                });
           
           return jacobianConstraints;
        }
                
        private JacobianConstraint[] GetJacobianConstraints(
            CollisionPointStructure[] collisionPointsStruct,
            List<IConstraint> simulationJointList,
            IShape[] simulationObjs,
            PhysicsEngineParameters simulationParameters)
        {
            List<JacobianConstraint> jacobianConstraints = new List<JacobianConstraint>();

            ////Collision Contact
            jacobianConstraints.AddRange(
                    GetContactJacobianConstraints(
                        collisionPointsStruct,
                        simulationObjs));

            ////Joints
            if (simulationJointList.Count > 0)
            {
                List<JacobianConstraint>[] jConstraints = new List<JacobianConstraint>[simulationJointList.Count];
                var rangePartitioner = Partitioner.Create(0, simulationJointList.Count, Convert.ToInt32(simulationJointList.Count / EngineParameters.MaxThreadNumber) + 1);

                Parallel.ForEach(
                    rangePartitioner,
                    new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    (range, loopState) =>
                    {
                        for (int i = range.Item1; i < range.Item2; i++)
                            jConstraints[i] = simulationJointList[i].BuildJacobian();
                    });

                jacobianConstraints.AddRange(jConstraints.SelectMany(f => f));
            }
            
            return jacobianConstraints.ToArray();
        }

        private List<JacobianConstraint> GetSoftBodyConstraints()
		{
			var constraints = new List<JacobianConstraint>();

			foreach (ISoftShape softShape in SoftShapes)
			{
				foreach (var item in softShape.SoftConstraint)
					constraints.AddRange(item.BuildJacobian());
			}

			return constraints;
		}

		private List<JacobianConstraint> GetJacobianJointConstraint(
			List<IConstraint> simulationJointList,
			IShape[] simulationObjs,
			double? stabilizationCoeff = null)
		{
			var constraint = new List<JacobianConstraint>();
						
			if (stabilizationCoeff.HasValue)
			{
					foreach (var constraintItem in simulationJointList)
					constraint.AddRange(constraintItem.BuildJacobian(stabilizationCoeff));
			}
			else
			{
				foreach (var constraintItem in simulationJointList)
					constraint.AddRange(constraintItem.BuildJacobian());
			}

			return constraint;
		}

		#endregion

		#region Solver Matrix Builder

		private double[] BuildMatrixAndExecuteSolver(
			JacobianConstraint[] contactConstraints,
			LinearProblemProperties linearProblemProperties,
			int nIterations)
		{
			if (linearProblemProperties != null)
			{
				Solver.GetSolverParameters().SetSolverMaxIteration(nIterations);

				double[]  solutionValues = Solver.Solve(linearProblemProperties, new double[linearProblemProperties.Count]);

				for (int j = 0; j < contactConstraints.Length; j++)
				{
					contactConstraints[j].StartImpulse.SetStartValue(solutionValues[j]);
				}

				return solutionValues;
			}

			return null;
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

