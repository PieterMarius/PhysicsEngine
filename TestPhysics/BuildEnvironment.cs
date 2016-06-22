using System;
using CollisionEngine;
using LCPSolver;
using MonoPhysicsEngine;

namespace TestPhysics
{
	public class BuildEnvironment
	{
		public BuildEnvironment()
		{
			
		}

		public PhysicsEngine GetPhysicsEnvironment()
		{
			SimulationParameters simulationParam = new SimulationParameters();
			SolverParameters solverParameters = new SolverParameters();
			CollisionEngineParameters collisionEngineParam = new CollisionEngineParameters();

			PhysicsEngine physicsEnvironment = new PhysicsEngine(
													simulationParam,
													collisionEngineParam,
													solverParameters);

			return physicsEnvironment;
		}
	}
}

