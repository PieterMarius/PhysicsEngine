using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public class SimulationParameters
	{
		#region Simulation Parameters Properties

		/// <summary>
		/// Gets the time step.
		/// </summary>
		/// <value>The time step.</value>
		public double TimeStep { get; private set; }

		/// <summary>
		/// Gets the CFM stabilization parameter.
		/// </summary>
		/// <value>The CF.</value>
		public double CFM { get; private set; }

		/// <summary>
		/// Gets the baumgarte stabilization parameter.
		/// </summary>
		/// <value>The baum stabilization.</value>
		public double BaumStabilization { get; private set; }

		/// <summary>
		/// Gets the shift parmeters to static friction tolerance.
		/// </summary>
		/// <value>The shift to static friction tolerance.</value>
		public double ShiftToStaticFrictionTolerance { get; private set; }

		/// <summary>
		/// Gets the collision distance.
		/// </summary>
		/// <value>The collision distance.</value>
		public double CollisionDistance { get; private set; }

		/// <summary>
		/// Gets the external force.
		/// </summary>
		/// <value>The external force.</value>
		public Vector3 ExternalForce { get; private set; }

		/// <summary>
		/// Gets the normal collision iterations.
		/// </summary>
		/// <value>The normal collision iterations.</value>
		public int NormalCollisionIterations { get; private set; }

		/// <summary>
		/// Gets the friction and normal iterations.
		/// </summary>
		/// <value>The friction and normal iterations.</value>
		public int FrictionAndNormalIterations { get; private set; }

		/// <summary>
		/// Gets the joints iterations.
		/// </summary>
		/// <value>The joints iterations.</value>
		public int JointsIterations { get; private set; }

		/// <summary>
		/// Gets the overall constraints iterations.
		/// </summary>
		/// <value>The overall constraints iterations.</value>
		public int OverallConstraintsIterations { get; private set; }

		/// <summary>
		/// Gets the linear velocity tolerance for object disabling.
		/// </summary>
		/// <value>The linear vel disable.</value>
		public double LinearVelDisable { get; private set; }

		/// <summary>
		/// Gets the angular velocity tolerance for object disable.
		/// </summary>
		/// <value>The angular vel disable.</value>
		public double AngularVelDisable { get; private set; }

		/// <summary>
		/// Discrete Continuos Collision Detection.
		/// </summary>
		/// <value><c>true</c> if discrete CC; otherwise, <c>false</c>.</value>
		public bool DiscreteCCD { get; private set; }

		/// <summary>
		/// Gets the compenetration tolerance.
		/// </summary>
		/// <value>The compenetration tolerance.</value>
		public double CompenetrationTolerance { get; private set; }

		/// <summary>
		/// Gets the velocity tolerance.
		/// </summary>
		/// <value>The velocity tolerance.</value>
		public double VelocityTolerance { get; private set; }

		/// <summary>
		/// Gets the inertia parameter.
		/// </summary>
		/// <value>The inertia parameter.</value>
		public double InertiaParameter { get; private set; }

		/// <summary>
		/// Gets the max thread number.
		/// </summary>
		/// <value>The max thread number.</value>
		public int MaxThreadNumber { get; private set; }

		/// <summary>
		/// Gets the warm-starting value.
		/// </summary>
		/// <value>The warm starting value.</value>
		public double WarmStartingValue { get; private set; }

		#endregion

		#region Constructors

		public SimulationParameters ()
		{
			TimeStep = 0.015;
			CFM = 0.001;
			BaumStabilization = 20;
			NormalCollisionIterations = 25;
			FrictionAndNormalIterations = 15;
			JointsIterations = 20;
			OverallConstraintsIterations = 8;
			LinearVelDisable = 0.0;
			AngularVelDisable = 0.0;
			ShiftToStaticFrictionTolerance = 0.001;
			DiscreteCCD = false;
			CollisionDistance = 0.009;
			CompenetrationTolerance = 0.001;
			VelocityTolerance = 0.01;
			ExternalForce = new Vector3(0.0, -9.81, 0.0);
			InertiaParameter = -0.0009;
			WarmStartingValue = 0.85;
			MaxThreadNumber = 2;
		}

		public SimulationParameters (
			double timeStep,
			double cfm,
			double baumStabilization,
			double linearVelDisable,
			double angularVelDisable,
			double shiftToStaticFrictionTolerance,
			bool discreteCCD,
			double collisionDistance,
			double compenetrationTolerance,
			double warmStartiongValue,
			Vector3 externalForce,
			int maxThreadNumber)
		{
			TimeStep = timeStep;
			CFM = cfm;
			BaumStabilization = baumStabilization;
			LinearVelDisable = linearVelDisable;
			AngularVelDisable = angularVelDisable;
			ShiftToStaticFrictionTolerance = shiftToStaticFrictionTolerance;
			DiscreteCCD = discreteCCD;
			CollisionDistance = collisionDistance;
			CompenetrationTolerance = compenetrationTolerance;
			ExternalForce = externalForce;
			WarmStartingValue = warmStartiongValue;
			MaxThreadNumber = maxThreadNumber;
		}
			
		#endregion

		#region Public Methods

		public void SetTimeStep(double timeStep)
		{
			TimeStep = timeStep;
		}

		public void SetCFM(double CFM)
		{
			this.CFM = CFM;
		}

		public void SetBaumStabilization(double baumStabilization)
		{
			BaumStabilization = baumStabilization;
		}

		public void SetLinearVelDisable(double linearVelDisable)
		{
			LinearVelDisable = linearVelDisable;
		}

		public void SetAngularVelDisable(double angularVelDisable)
		{
			AngularVelDisable = angularVelDisable;
		}

		public void SetShiftToStaticFrictionTolerance(double shiftToStaticFrictionTolerance)
		{
			ShiftToStaticFrictionTolerance = shiftToStaticFrictionTolerance;
		}

		public void SetExternalForce(Vector3 externalForce)
		{
			ExternalForce = externalForce;
		}

		#endregion
	}
}

