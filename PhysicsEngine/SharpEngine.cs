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
using System.Runtime.InteropServices;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ContactPartitioning;
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.Wrapper;
using SharpPhysicsEngine.Wrapper.Joint;
using System.Collections.Concurrent;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ContinuosCollisionDetection;
using SharpPhysicsEngine.SolutionIntegration;
using SharpPhysicsEngine.CollisionEngine.Dynamic_Bounding_Tree;
using static SharpPhysicsEngine.Helper.CommonUtilities;

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
        		

        private Dictionary<int, StabilizationValues> PreviousShapesProperties;

		/// <summary>
		/// The contact partitioning engine.
		/// </summary>
		private readonly IContactPartitioningEngine contactPartitioningEngine;

		/// <summary>
		/// The last execution solver error.
		/// </summary>
		double solverError;
                
        /// <summary>
        /// Accumulated timestep
        /// </summary>
        double partialTimeStep;

        /// <summary>
        /// Used for Continuos Collision Detection
        /// </summary>
        private double GlobalTimestep;
                
        /// <summary>
        /// Hierarchical Tree of Simulation Object
        /// </summary>
        //private AABBTree HierarchicalTree;


        #region Support Engines

        /// <summary>
        /// The solver.
        /// </summary>
        private ISolver Solver;

        /// <summary>
        /// Generate ID for shape of simulation
        /// </summary>
        private readonly HashGenerator HsGenerator;

        /// <summary>
        /// Build the linear system: Ax = b (symmetric)
        /// </summary>
		private readonly LinearProblemBuilderEngine LinearSystemBuilder;

        /// <summary>
        /// Update objects position and integrate external forces
        /// </summary>
		private readonly IntegratePosition IntegratePositionEngine;

        /// <summary>
        /// Update velocity related to Collisions and Joints constraints solutions
        /// </summary>
        private readonly IntegrateVelocity IntegrateVelocityEngine;

        /// <summary>
        /// 
        /// </summary>
		private readonly ContactConstraintBuilder contactConstraintBuilder;

        /// <summary>
        /// 
        /// </summary>
        private readonly WarmStartEngine warmStartEngine;

        /// <summary>
        /// 
        /// </summary>
        private readonly RayCastingEngine rayCastEngine;

        /// <summary>
        /// 
        /// </summary>
        private readonly ICCDEngine ccdEngine;

        private SolverType solverType;

        #endregion

        #endregion
                
        #region Constructor

        public SharpEngine (
			PhysicsEngineParameters simulationParameters,
			CollisionEngineParameters collisionEngineParameters,
			SolverParameters solverParameters)
		{
			SolverParameters = solverParameters;
            CollisionEngineParam = collisionEngineParameters;
            EngineParameters = simulationParameters;

            // Default solver
            SetSolverType(SolverType.ProjectedGaussSeidel);

            Shapes = new IShape[0];

            CollisionEngine = new CollisionDetectionEngine(
                collisionEngineParameters, 
                EngineParameters.CollisionDistance);

			contactPartitioningEngine = new ContactPartitioningEngine();
            			
            CollisionShapes = new List<ICollisionShape>();
            CollisionJoints = new List<ICollisionJoint>();
			Joints = new List<IConstraint> ();
			HsGenerator = new HashGenerator();
			LinearSystemBuilder = new LinearProblemBuilderEngine(EngineParameters);
            IntegrateVelocityEngine = new IntegrateVelocity(EngineParameters);
			IntegratePositionEngine = new IntegratePosition(EngineParameters);
			contactConstraintBuilder = new ContactConstraintBuilder(EngineParameters);
            rayCastEngine = new RayCastingEngine();
            warmStartEngine = new WarmStartEngine(EngineParameters);
            ccdEngine = new ConservativeAdvancement();
        }

		public SharpEngine()
			: this(
                  new PhysicsEngineParameters(), 
                  new CollisionEngineParameters(), 
                  new SolverParameters())
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
                softShape.SetID(HsGenerator.GetHash());
                foreach (var point in softShape.ShapePoints)
                    point.SetID(HsGenerator.GetHash());
            }
            else
                simObj.SetID(HsGenerator.GetHash());

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

			SoftShapes = Shapes.Where(x => (x as ISoftShape) != null)
                               .Cast<ISoftShape>()
                               .ToArray();

            CollisionEngine.AddShape(simObj);
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

                    CollisionEngine.RemoveShape(bufferList[shapeIndex]);

					bufferList.RemoveAt(shapeIndex);
					Shapes = bufferList.ToArray();

					#endregion
				}
			}
		}

		public void RemoveAllShapes()
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

        public List<Tuple<Vector3d, Vector3d>> GetHierarchicalTreeAABB()
        {
            var result = new List<Tuple<Vector3d, Vector3d>>();
            var nodes = CollisionEngine.GetHierarchicalTree();

            if (nodes != null)
            {
                for (int i = 0; i < nodes.Count; i++)
                    result.Add(new Tuple<Vector3d, Vector3d>(nodes[i].aabb.Min, nodes[i].aabb.Max));
                
                return result;
            }

            return null;
        }

        //public List<Tuple<Vector3d, Vector3d>> GetShapesAABB()
        //{
        //    var result = new List<Tuple<Vector3d, Vector3d>>();

        //    for (int i = 0; i < Shapes.Length; i++)
        //    {
        //        if (Shapes[i] is ShapeDefinition.ConvexShape)
        //            result.Add(new Tuple<Vector3d, Vector3d>(Shapes[i].AABBox.Min, Shapes[i].AABBox.Max));
        //        else if (Shapes[i] is ShapeDefinition.ConcaveShape)
        //        {
        //            var shapesGeom = ((ShapeDefinition.ConcaveShape)Shapes[i]).ConvexShapesGeometry;

        //            for (int j = 0; j < shapesGeom.Length; j++)
        //            {
        //                result.Add(new Tuple<Vector3d, Vector3d>(shapesGeom[j].AABBox.Min, shapesGeom[j].AABBox.Max));
        //            }
        //            result.Add(new Tuple<Vector3d, Vector3d>(Shapes[i].AABBox.Min, Shapes[i].AABBox.Max));
        //        }
        //    }

        //    return result;
        //}

        //TODO Test Hierarchical tree intersection
        //public List<Tuple<Vector3d, Vector3d>> GetHierarchicalIntersection()
        //{
        //    var result = new List<Tuple<Vector3d, Vector3d>>();
        //    int height = HierarchicalTree.GetMaxHeight();
            
        //    for (int i = 0; i < Shapes.Length; i++)
        //    {
        //        var overlaps = HierarchicalTree.QueryOverlaps(ExtractIAABBFromShape(Shapes[i]));
        //        foreach (var item in overlaps)
        //        {
        //            var aabb = item.GetAABB();
        //            result.Add(new Tuple<Vector3d, Vector3d>(aabb.Min, aabb.Max));
        //        }
        //    }
        //    return result;
        //}

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
            CollisionDetection();

			if (collisionPoints == null)
				return new List<CollisionPointStructure>();
			
			return new List<CollisionPointStructure>(collisionPoints);
		}


        /// <summary>
        /// Return the nearest shape ID, if ray doesn't collide returns null
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public int? RayCastShapeID(
            Vector3d origin,
            Vector3d direction)
        {
            return rayCastEngine.ExecuteWithID(Shapes, origin, direction);
        }

        public Vector3d? RayCastShape(
            Vector3d origin,
            Vector3d direction)
        {
            return rayCastEngine.Execute(Shapes, origin, direction);
        }

        #endregion

        #region Solver

        public double GetSolverError()
		{
			return solverError;
		}

		public void SetSolverType(SolverType type)
		{
			switch (type)
			{
				case SolverType.NonLinearConjugateGradient:
					Solver = new NonLinearConjugateGradient(SolverParameters);
                    solverType = SolverType.NonLinearConjugateGradient;
                    break;
                    				
                case SolverType.RedBlackProjectedGaussSeidel:
                    Solver = new RedBlackProjectedGaussSeidel(SolverParameters);
                    solverType = SolverType.RedBlackProjectedGaussSeidel;
                    break;

                case SolverType.FischerNewton:
                    Solver = new FischerNewton(SolverParameters);
                    solverType = SolverType.FischerNewton;
                    break;

                case SolverType.Lemke:
                    Solver = new Lemke(SolverParameters);
                    solverType = SolverType.Lemke;
                    break;

                case SolverType.ProjectedSymmetricGS:
                    Solver = new ProjectedSymmetricGS(SolverParameters, LinearSystemBuilder);
                    solverType = SolverType.ProjectedSymmetricGS;
                    break;

                case SolverType.NonLinearGaussSeidel:
                    Solver = new NonLinearGaussSeidel(SolverParameters, LinearSystemBuilder, IntegrateVelocityEngine);
                    solverType = SolverType.NonLinearGaussSeidel;
                    break;

                case SolverType.ProjectedGaussSeidel:
                default:
                    Solver = new ProjectedGaussSeidel(SolverParameters);
                    solverType = SolverType.ProjectedGaussSeidel;
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
            GlobalTimestep = timeStep;
			TimeStep = timeStep;

            partialTimeStep = 0.0;

            if (EngineParameters.CCD)
            {
                while (partialTimeStep < GlobalTimestep)
                    ExecuteFlow();
            }
            else
                ExecuteFlow();
        }

		public void Simulate()
		{
			Simulate(EngineParameters.TimeStep);
		}
        		

        #endregion

        #endregion

        #region Private Methods

        private void ExecuteFlow()
        {
            if (SolverParameters.NewtonStepInterations > 1)
            {
                double newtonTimeStep = GlobalTimestep / SolverParameters.NewtonStepInterations;
                TimeStep = newtonTimeStep;

                for (int i = 0; i < SolverParameters.NewtonStepInterations; i++)
                {
                    // Newton Iteration Step
                    CollisionDetection();
                    PartitionEngineExecute();
                    SolveVelocityConstraints();
                    IntegratePositionEngine.IntegrateObjectsPosition(ref Shapes, TimeStep, false);

                    if (i + 1 == SolverParameters.NewtonStepInterations)
                        IntegratePositionEngine.AddExternalForce(ref Shapes, GlobalTimestep);
                    
                    UpdateHierarchicalTree();
                }
            }
            else
            {
                CollisionDetection();
                PartitionEngineExecute();
                SolveVelocityConstraints();
                IntegratePositionEngine.IntegrateObjectsPosition(ref Shapes, TimeStep, true);
                UpdateHierarchicalTree();
            }
        }
                
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

		private void SolveVelocityConstraints()
		{
            #region Contact and Joint elaboration

            //SaveShapePreviousProperties(collisionPoints);
            
            if (Partitions != null && Partitions.Any()) 
			{
                var rangePartitioner = Partitioner.Create(
                    0,
                    Partitions.Count,
                    Convert.ToInt32(Partitions.Count / EngineParameters.MaxThreadNumber) + 1);

                Parallel.ForEach(
                    rangePartitioner,
                    new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    (range, loopState) =>
                    {
                        for (int i = range.Item1; i < range.Item2; i++)
                        {
                            JacobianConstraint[] jacobianConstraints = GetJacobianConstraints(
                                                                        Partitions[i].PartitionedCollisionPoints.ToArray(),
                                                                        Partitions[i].PartitionedJoints,
                                                                        Shapes,
                                                                        EngineParameters);

                            if (jacobianConstraints.Length > 0)
                            {
                                LinearProblemProperties LCP = GenerateLCP(jacobianConstraints);
                                double[]  overallSolution = Solver.Solve(LCP, jacobianConstraints, LCP.StartImpulse);
                                //double[] overallSolution = Solve(jacobianConstraints);

                                IntegrateVelocityEngine.UpdateVelocity(jacobianConstraints, overallSolution);
                            }
                        }
                    });
			}

            #endregion
        }

		#region Collision Detection

        private void SaveShapePreviousProperties(CollisionPointStructure[] collisionPointsStruct)
        {
            PreviousShapesProperties = new Dictionary<int, StabilizationValues>();

            foreach (var item in collisionPointsStruct)
            {
                var shapeA = Shapes.First(x => x.ID == item.ObjectIndexA);
                var shapeB = Shapes.First(x => x.ID == item.ObjectIndexB);

                if (!(shapeA is SimSoftShape) &&
                    !(shapeB is SimSoftShape))
                {

                    if (!PreviousShapesProperties.ContainsKey(item.ObjectIndexA))
                        PreviousShapesProperties.Add(item.ObjectIndexA, new StabilizationValues(
                                                    shapeA.Position,
                                                    shapeA.RotationStatus,
                                                    shapeA.LinearVelocity,
                                                    shapeA.AngularVelocity));

                    if (!PreviousShapesProperties.ContainsKey(item.ObjectIndexB))
                        PreviousShapesProperties.Add(item.ObjectIndexB, new StabilizationValues(
                                                    shapeB.Position,
                                                    shapeB.RotationStatus,
                                                    shapeB.LinearVelocity,
                                                    shapeB.AngularVelocity));
                }
            }
        }

		/// <summary>
		/// Collisions detection.
		/// </summary>
		private void CollisionDetection()
		{
			List<CollisionPointStructure> previousCollisionPoints = null;

			if (collisionPoints != null &&
				collisionPoints.Length > 0)
                previousCollisionPoints = new List<CollisionPointStructure>(collisionPoints);

            //Creo l'array contenente la geometria degli oggetti
            IShape[] simShapes = Array.ConvertAll(
                            Shapes,
                            item => (item.ExcludeFromCollisionDetection) ? null : item);
                        
            //var collisionPair = ContinuosCollisionDetection(simShapes);

            if (EngineParameters.CCD)
                GlobalCCDSimulation(simShapes);

            //Eseguo il motore che gestisce le collisioni
            //double collisionDist = CollisionEngineParam.CollisionDistance;

            //collisionDist = 100.0;

            // var actualCollisionPoints = CollisionEngine.Execute(simShapes, collisionPair, collisionDist);

            var actualCollisionPoints = CollisionEngine.Execute(simShapes);

            //var warmStartedPoints = warmStartEngine.GetWarmStartedCollisionPoints(
            //    Shapes, 
            //    previousCollisionPoints, 
            //    actualCollisionPoints, 
            //    PreviousShapesProperties);

            collisionPoints = actualCollisionPoints.ToArray();
        }

        private void GlobalCCDSimulation(IShape[] simShapes)
        {
            double lowerTimeStep = GlobalTimestep;

            for (int i = 0; i < simShapes.Length; i++)
            {
                for (int j = i + 1; j < simShapes.Length; j++)
                {
                    double? timeOfImpact = ccdEngine.GetAABBTimeOfImpact(
                        simShapes[i],
                        simShapes[j],
                        TimeStep);
                    
                    if (timeOfImpact.HasValue)
                        lowerTimeStep = timeOfImpact.Value;
                }
            }

            if (lowerTimeStep < 0.004)
                TimeStep = 0.004;
            else
                TimeStep = lowerTimeStep;

            partialTimeStep += TimeStep;
        }

        private List<CollisionPair> ContinuosCollisionDetection(IShape[] simShapes)
        {
            var collisionPair = new List<CollisionPair>();

            var selection = simShapes.Select((v, i) => new { value = v, index = i })
                                     .Where(x => x.value.ActiveCCD)
                                     .ToList();

            double lowerTimeStep = 0.016;

            for (int i = 0; i < simShapes.Length; i++)
            {
                for (int j = i + 1; j < simShapes.Length; j++)
                {
                    if (simShapes[i].AngularVelocity.Dot(simShapes[i].AngularVelocity) == 0.0 &&
                        simShapes[i].LinearVelocity.Dot(simShapes[i].LinearVelocity) == 0.0 &&
                        simShapes[j].AngularVelocity.Dot(simShapes[j].AngularVelocity) == 0.0 &&
                        simShapes[j].LinearVelocity.Dot(simShapes[j].LinearVelocity) == 0.0)
                        continue;

                    double? timeOfImpact = ccdEngine.GetAABBTimeOfImpact(
                        simShapes[i],
                        simShapes[j],
                        TimeStep);

                    //double? timeOfImpact = ccdEngine.GetTimeOfImpact(
                    //    simShapes[i], 
                    //    simShapes[j], 
                    //    TimeStep);

                    if (timeOfImpact.HasValue &&
                        timeOfImpact < lowerTimeStep)
                    {
                        lowerTimeStep = timeOfImpact.Value;

                        collisionPair.Add(new CollisionPair(i, j));
                    }
                }
            }

            if (lowerTimeStep < 0.004)
                TimeStep = 0.004;
            else
                TimeStep = lowerTimeStep;

            partialTimeStep += TimeStep;

            Console.WriteLine("TimeStep " + TimeStep);

            return collisionPair;
        }

        private void UpdateHierarchicalTree()
        {
            foreach(var shape in Shapes)
            {
                if (!shape.IsStatic)
                    CollisionEngine.UpdateShape(shape);
            }
        }

		#endregion

		#region Contact Partitioning

		private void PartitionEngineExecute()
		{
			Partitions = contactPartitioningEngine.GetPartitions(
													        collisionPoints,
													        Joints,
													        Shapes,
                                                            SoftShapes);
		}

        #endregion

        #region Jacobian Constraint

        private JacobianConstraint[] GetJacobianConstraints(
            CollisionPointStructure[] collisionPointsStruct,
            List<IConstraint> simulationJointList,
            IShape[] simulationObjs,
            PhysicsEngineParameters simulationParameters)
        {
            List<JacobianConstraint> jacobianConstraints = new List<JacobianConstraint>();

            ////Collision Contact
            jacobianConstraints.AddRange(
                    GetCollisionConstraints(
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

        private List<JacobianConstraint> GetCollisionConstraints(
            CollisionPointStructure[] collisionPointsStruct,
            IShape[] simulationObjs)
        {
            List<JacobianConstraint> jacobianConstraints = new List<JacobianConstraint>();

            if (collisionPointsStruct.Length > 0)
            {
                var rangePartitioner = Partitioner.Create(0, collisionPointsStruct.Length, Convert.ToInt32(collisionPointsStruct.Length / EngineParameters.MaxThreadNumber) + 1);
                
                var sync = new object();

                Parallel.ForEach(
                    rangePartitioner,
                    new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    (range, loopState) =>
                    {
                        for (int i = range.Item1; i < range.Item2; i++)
                        {
                            var item = collisionPointsStruct[i];

                            IShape objectA = simulationObjs.First(x => x.ID == item.ObjectIndexA);
                            IShape objectB = simulationObjs.First(x => x.ID == item.ObjectIndexB);

                            List<JacobianConstraint> constraintsBuf = contactConstraintBuilder.BuildContactConstraint(
                                item,  
                                objectA, 
                                objectB);

                            lock (sync)
                            {
                                int idx = jacobianConstraints.Count;
                                if (idx > 0)
                                {
                                    for (int j = 0; j < constraintsBuf.Count; j++)
                                    {
                                        var constraintBuf = constraintsBuf[j];
                                        if (constraintBuf.Type == ConstraintType.Friction)
                                        {
                                            constraintBuf.ContactReference = constraintBuf.ContactReference + idx;
                                            constraintsBuf[j] = new JacobianConstraint(constraintBuf);
                                        }
                                    }
                                }

                                jacobianConstraints.AddRange(constraintsBuf);

                            }
                        }
                    });
            }
           
           return jacobianConstraints;
        }

        private LinearProblemProperties GenerateLCP(JacobianConstraint[] jacobianConstraints)
        {
            switch(solverType)
            {
                case SolverType.Lemke:
                    var baseLCP = LinearSystemBuilder.BuildLCP(jacobianConstraints, TimeStep);
                    return LinearSystemBuilder.GetLCPFrictionMatrix(baseLCP);

                case SolverType.ProjectedSymmetricGS:
                case SolverType.ProjectedGaussSeidel:
                case SolverType.NonLinearConjugateGradient:
                case SolverType.RedBlackProjectedGaussSeidel:
                default:
                    return LinearSystemBuilder.BuildLCP(jacobianConstraints, TimeStep);
            }
        }



        private double[] Solve(JacobianConstraint[] jacobianConstraints)
        {
            LinearProblemProperties LCP = LinearSystemBuilder.BuildLCP(jacobianConstraints, 0.001);
            double[] partialSolution = new double[LCP.Count];
            double[] overallSolution = new double[LCP.Count];

            Solver.GetSolverParameters().SetSolverMaxIteration(2);

            for (int i = 0; i < 15; i++)
            {
                overallSolution = MathUtils.Add(overallSolution, partialSolution);
                partialSolution = Solver.Solve(LCP, jacobianConstraints, overallSolution);

                
            }

            //double[] overallSolution = Solver.Solve(LCP, LCP.StartImpulse);

            return overallSolution;
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

		//private List<JacobianConstraint> GetJacobianJointConstraint(
		//	List<IConstraint> simulationJointList,
		//	IShape[] simulationObjs,
		//	double? stabilizationCoeff = null)
		//{
		//	var constraint = new List<JacobianConstraint>();
						
		//	if (stabilizationCoeff.HasValue)
		//	{
		//			foreach (var constraintItem in simulationJointList)
		//			constraint.AddRange(constraintItem.BuildJacobian(TimeStep, stabilizationCoeff));
		//	}
		//	else
		//	{
		//		foreach (var constraintItem in simulationJointList)
		//			constraint.AddRange(constraintItem.BuildJacobian(TimeStep));
		//	}

		//	return constraint;
		//}

		#endregion

		#region Solver Matrix Builder
                
        

        

        public static double ComputeSolverError(
            double[] x,
            double[] oldx)
        {
            double[] actualSolverErrorDiff = MathUtils.Minus(x, oldx);
            return MathUtils.Dot(actualSolverErrorDiff, actualSolverErrorDiff);
        }

        private double[] BuildMatrixAndExecuteSolver(
			JacobianConstraint[] contactConstraints,
			LinearProblemProperties linearProblemProperties,
			int nIterations)
		{
			if (linearProblemProperties != null)
			{
				Solver.GetSolverParameters().SetSolverMaxIteration(nIterations);

				double[]  solutionValues = Solver.Solve(linearProblemProperties, contactConstraints, new double[linearProblemProperties.Count]);
                				
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

