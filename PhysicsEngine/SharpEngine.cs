using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Threading.Tasks;
using PhysicsEngineMathUtility;
using ShapeDefinition;
using CollisionEngine;
using LCPSolver;

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
		/// The simulation objects.
		/// </summary>
		IShape[] Shapes;

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
                if (collisionPartitionedPoints != null)
                {
                    double baumgarteStabilizationValue = 1.0 / TimeStep;

                    for (int i = 0; i < collisionPartitionedPoints.Count; i++)
                    {
                        if (EngineParameters.PositionBasedJointIterations > 0)
                        {
                            JacobianConstraint[] jointConstraints = GetJacobianJointConstraint(
                                                                       partitionedJoint[i],
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
            
            if (collisionPartitionedPoints != null) 
			{
                bool convertSetting = false;
                if (EngineParameters.PositionStabilization)
                {
                    EngineParameters.SetPositionStabilization(false);
                    convertSetting = true;
                }

                for (int i = 0; i < collisionPartitionedPoints.Count;i++)
				{
                    JacobianConstraint[] jacobianConstraints = GetJacobianConstraint(
                                                                   collisionPartitionedPoints[i].ToArray(),
																   partitionedJoint[i],
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

                        jacobianConstraints = ContactSorting(jacobianConstraints);

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

		//private void WarmStarting(List<CollisionPointStructure> collisionPointsBuffer)
		//{
		//	foreach (CollisionPointStructure cPoint in collisionPointsBuffer)
		//	{
  //              //TODO Work in progress
  //              int pointBufferIndex = collisionPoints.ToList().FindIndex(
  //                                    x => (x.ObjectA == cPoint.ObjectA &&
  //                                          x.ObjectB == cPoint.ObjectB) ||
  //                                         (x.ObjectA == cPoint.ObjectB &&
  //                                          x.ObjectB == cPoint.ObjectA));

                

		//		if (pointBufferIndex > -1)
		//		{
		//			CollisionPointStructure pointBuffer = collisionPoints[pointBufferIndex];

  //                  if((pointBuffer.CollisionPoint.CollisionPointA - cPoint.CollisionPoint.CollisionPointA).Length() < 
  //                      0.001)
  //                  {
  //                      if (cPoint.FrameCount > 5)
  //                      {
  //                          collisionPoints[pointBufferIndex].CollisionPoint = cPoint.CollisionPoint;
  //                          collisionPoints[pointBufferIndex].CollisionPoints = cPoint.CollisionPoints;
  //                          //collisionPoints[pointBufferIndex].SetIntersection(cPoint.Intersection);
  //                          //collisionPoints[pointBufferIndex].SetObjectDistance(cPoint.ObjectDistance);
  //                      }

  //                      collisionPoints[pointBufferIndex].SetFrameCount(cPoint.FrameCount + 1);
  //                  }
  //                  else
  //                  {
  //                      collisionPoints[pointBufferIndex].SetFrameCount(0);
  //                  }


  //   //               for (int i = 0; i < pointBuffer.CollisionPoints.Count(); i++)
		//			//{
		//			//	int ppBuffer = cPoint.CollisionPoints.ToList().FindIndex(x => 	(Vector3.Length(x.CollisionPointA -
		//			//																			pointBuffer.CollisionPoints[i].CollisionPointA) < 0.01 &&
		//			//																	Vector3.Length(x.CollisionPointB -
		//			//																			pointBuffer.CollisionPoints[i].CollisionPointB) < 0.01) ||
		//			//																	(Vector3.Length(x.CollisionPointA -
		//			//																			pointBuffer.CollisionPoints[i].CollisionPointB) < 0.01 &&
		//			//																	Vector3.Length(x.CollisionPointB -
		//		 //                                                                               pointBuffer.CollisionPoints[i].CollisionPointA) < 0.01));

		//			//	if (ppBuffer > -1)
		//			//	{
		//			//		collisionPoints[pointBufferIndex].CollisionPoints[i].StartImpulseValue[0].SetStartValue(cPoint.CollisionPoints[ppBuffer].StartImpulseValue[0].StartImpulseValue);
		//			//		collisionPoints[pointBufferIndex].CollisionPoints[i].StartImpulseValue[1].SetStartValue(cPoint.CollisionPoints[ppBuffer].StartImpulseValue[1].StartImpulseValue);
		//			//		collisionPoints[pointBufferIndex].CollisionPoints[i].StartImpulseValue[2].SetStartValue(cPoint.CollisionPoints[ppBuffer].StartImpulseValue[2].StartImpulseValue);
		//			//	}
		//			//}
		//		}
		//	}
		//}

		#endregion

		#region Contact Partitioning

		private void PartitionEngineExecute()
		{
            collisionPartitionedPoints = null;
                        
            List<SpatialPartition> partitions = contactPartitioningEngine.CalculateSpatialPartitioning(
													collisionPoints,
													Joints,
													Shapes);

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
							IConstraint smJoint = Joints.Find(item =>
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

		public List<JacobianConstraint> GetJacobianConstraint(
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

            #region Soft Body Constraint

            //foreach(IShape shape in simulationObjs)

            #endregion

            return constraint;
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

		/// <summary>
		/// Builds the LCP matrix for solver.
		/// </summary>
		private LinearProblemProperties BuildLCPMatrix(
			JacobianConstraint[] contact,
			bool positionStabilization = false)
		{
			if (contact.Length > 0) 
			{
				SparseElement[] M = new SparseElement[contact.Length];
				double[] B = new double[contact.Length];
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

                Parallel.For(0,
                    contact.Length,
                    new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    i =>
                    {
                        JacobianConstraint contactA = contact[i];

                        if (positionStabilization)
                            B[i] = contactA.CorrectionValue;
                        else
                            B[i] = -(contactA.B - ((contactA.CorrectionValue) < 0 ? Math.Max(contactA.CorrectionValue, -EngineParameters.MaxCorrectionValue) :
                                                                                    Math.Min(contactA.CorrectionValue, EngineParameters.MaxCorrectionValue)));

                        if (contactA.ContactReference.HasValue)
                            constraints[i].Add(contactA.ContactReference);

                        constraintsLimit[i] = contactA.ConstraintLimit;
                        constraintsType[i] = contactA.Type;

                        double mValue = addLCPValue(contactA,
                                                    contactA);

                        //Diagonal value
                        mValue += contactA.CFM +
                                  EngineParameters.CFM +
                                  1E-40;

                        D[i] = 1.0 / mValue;

                        for (int j = i + 1; j < contact.Length; j++)
                        {
                            JacobianConstraint contactB = contact[j];
                            
                            if (contactA.ObjectA.GetID() == contactB.ObjectA.GetID() ||
                                contactA.ObjectB.GetID() == contactB.ObjectB.GetID() ||
                                contactA.ObjectA.GetID() == contactB.ObjectB.GetID() ||
                                contactA.ObjectB.GetID() == contactB.ObjectA.GetID())
                            {
                                if (contactA.Type == contactB.Type &&
                                    contactB.Type == ConstraintType.Collision &&
                                    contactA.ObjectA.GetID() == contactB.ObjectA.GetID() &&
                                    contactA.ObjectB.GetID() == contactB.ObjectB.GetID())
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
					D,
					constraintsLimit,
					constraintsType,
                    constraintsArray);
			}

			return null;
		}

		private double addLCPValue(
			JacobianConstraint contactA,
			JacobianConstraint contactB)
		{
            double linearA = 0.0;
			double angularA = 0.0;

			if (contactA.ObjectA.GetID() == contactB.ObjectA.GetID()) {

				linearA = contactA.LinearComponentA.Dot (
					contactB.LinearComponentA * contactA.ObjectA.InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
                    contactA.ObjectA.InertiaTensor * contactB.AngularComponentA);

			} else if (contactB.ObjectB.GetID() == contactA.ObjectA.GetID()) {

				linearA = contactA.LinearComponentA.Dot (
					contactB.LinearComponentB * contactA.ObjectA.InverseMass);
				
				angularA = contactA.AngularComponentA.Dot (
                    contactA.ObjectA.InertiaTensor * contactB.AngularComponentB);
			}

			double linearB = 0.0;
			double angularB = 0.0;

			if (contactB.ObjectA.GetID() == contactA.ObjectB.GetID()) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentA * contactA.ObjectB.InverseMass);
				
				angularB = contactA.AngularComponentB.Dot(
                    contactA.ObjectB.InertiaTensor * contactB.AngularComponentA);
				
			} else if (contactB.ObjectB.GetID() == contactA.ObjectB.GetID()) {
				
				linearB = contactA.LinearComponentB.Dot (
					contactB.LinearComponentB * contactA.ObjectB.InverseMass);
				
				angularB = contactA.AngularComponentB.Dot (
                    contactA.ObjectB.InertiaTensor * contactB.AngularComponentB);
			}

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
		/// <returns>The object velocity.</returns>
		/// <param name="simulationObj">Simulation object.</param>
		/// <param name="linearComponent">Linear component.</param>
		/// <param name="angularComponent">Angular component.</param>
		/// <param name="X">X.</param>
		/// <param name="index">Object index.</param>
		private void UpdateObjectVelocity(
			IShapeCommon simObj,
			Vector3 linearComponent,
			Vector3 angularComponent,
			double X)
		{
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

                simObj.SetLinearVelocity (linearVelocity);
                simObj.SetAngularVelocity (angularVelocity);
			}
		}

		/// <summary>
		/// Integrates the objects position.
		/// </summary>
		private void IntegrateObjectsPosition(
			IShape[] simulationObj)
		{
			int index = 0;
			foreach (IShape simObj in simulationObj) 
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
                        (TimeStep * EngineParameters.ExternalForce));

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

                    if (EngineParameters.SleepingObject)
                        ObjectSleep(simObj);

                    #endregion

                    #region Update AABB

                    if (ShapeDefinition.Helper.GetGeometry(simObj) != null &&
						(linearVelocity > 0.0 || angularVelocity > 0.0))
                    {
                       simObj.SetAABB();
                    }

                    #endregion

                }
				Shapes [index] = simObj;
				index++;
			}
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
				collisionPartitionedPoints = null;
				partitionedJoint = null;
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

