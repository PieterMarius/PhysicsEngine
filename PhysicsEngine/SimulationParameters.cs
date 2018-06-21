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

using SharpEngineMathUtility;

namespace SharpPhysicsEngine
{
	public class PhysicsEngineParameters
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
		/// Gets the normal CFM.
		/// </summary>
		/// <value>The normal CF.</value>
		public double NormalCFM { get; private set; }

		/// <summary>
		/// Gets the friction CFM.
		/// </summary>
		/// <value>The friction CF.</value>
		public double FrictionCFM { get; private set; }

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
		/// Gets the max thread number.
		/// </summary>
		/// <value>The max thread number.</value>
		public int MaxThreadNumber { get; private set; }

		/// <summary>
		/// Gets the warm-starting value.
		/// </summary>
		/// <value>The warm starting value.</value>
		public double WarmStartingValue { get; private set; }

		/// <summary>
		/// Gets the max correction value.
		/// </summary>
		/// <value>The max correction value.</value>
		public double MaxCorrectionValue { get; private set; }
        		
		/// <summary>
		/// Gets the baum position stabilization.
		/// </summary>
		/// <value>The baum position stabilization.</value>
		public double BaumPositionStabilization { get; private set; }

        /// <summary>
        /// Activate/Disable objects sleep
        /// </summary>
        public bool SleepingObject { get; private set; }

        /// <summary>
        /// Gets the number of frame under speed limit tolerance
        /// </summary>
        public int SleepingFrameLimit { get; private set; }

        /// <summary>
        /// Gets the successiveOverRelaxation term for object collision
        /// </summary>
        public double CollisionSORValue { get; private set; }

        /// <summary>
        /// Turn to zero if angular velocity is lower
        /// </summary>
        public double AngularVelocityMinLimit { get; private set; }

        #endregion

        #region Constructors

        public PhysicsEngineParameters ()
		{
			TimeStep = 0.01;
			CFM = 1E-07;
			NormalCFM = 0.0;
			FrictionCFM = 0.0;
			BaumStabilization = 1.0;
            LinearVelDisable = 0.08;
			AngularVelDisable = 0.1;
			ShiftToStaticFrictionTolerance = 0.05;
			DiscreteCCD = false;
			CollisionDistance = 0.003;
            CompenetrationTolerance = 0.05;
			ExternalForce = new Vector3(0.0, -9.81, 0.0);
			WarmStartingValue = 0.75;
			MaxThreadNumber = 4;
			MaxCorrectionValue = 10000.0;
			SleepingObject = false;
            SleepingFrameLimit = 7;
            CollisionSORValue = 0.5;
            AngularVelocityMinLimit = 0.0;
        }

		//TODO Update input parameters
		public PhysicsEngineParameters (
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

		public void SetBaumPositionStabilization(double baumStabilizationValue)
		{
			BaumPositionStabilization = baumStabilizationValue;
		}

        	#endregion
	}
}

