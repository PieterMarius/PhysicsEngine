using System;
using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public struct SimulationObject
	{

		#region Object status properties

		/// <summary>
		/// Mass of object.
		/// </summary>
		/// <value>The mass.</value>
		public double Mass{ get; private set; }

		/// <summary>
		/// Gets the inverse mass.
		/// </summary>
		/// <value>The inverse mass.</value>
		public double InverseMass{ get; private set; }

		/// <summary>
		/// Elastict coefficient.
		/// </summary>
		/// <value>The restitution coeff.</value>
		public double RestitutionCoeff{ get; private set; }

		/// <summary>
		/// Gets the static friction coeff.
		/// </summary>
		/// <value>The static friction coeff.</value>
		public double StaticFrictionCoeff{ get; private set; }

		/// <summary>
		/// Gets the dynamic friction coeff.
		/// </summary>
		/// <value>The dynamic friction coeff.</value>
		public double DynamicFrictionCoeff{ get; private set; }

		/// <summary>
		/// Gets the relative positions of each point of the object from initial mass center.
		/// </summary>
		/// <value>The relative positions.</value>
		public Vector3[] RelativePositions{ get; private set; }

		/// <summary>
		/// Gets the base inertia tensor ^(-1).
		/// </summary>
		/// <value>The inertia tensor.</value>
		public Matrix3x3 BaseInertiaTensor{ get; private set; }

		/// <summary>
		/// Gets the inertia tensor ^(-1).
		/// </summary>
		/// <value>The inverse inertia tensor.</value>
		public Matrix3x3 InertiaTensor{ get; private set; }

		/// <summary>
		/// Gets or sets the object geometry.
		/// </summary>
		/// <value>The object geometry.</value>
		public ObjectGeometry ObjectGeometry{ get; set; }

		/// <summary>
		/// Gets the type of the object.
		/// </summary>
		/// <value>The type of the object.</value>
		public ObjectType ObjectType { get; private set; }


		#endregion

		#region Object dynamic properties

		/// <summary>
		/// Gets the mass center position.
		/// </summary>
		/// <value>The position.</value>
		public Vector3 Position{ get; private set; }

		/// <summary>
		/// Gets the mass center start position.
		/// </summary>
		/// <value>The start position.</value>
		public Vector3 StartPosition{ get; private set; }

		/// <summary>
		/// Gets the actual linear velocity.
		/// </summary>
		/// <value>The linear velocity.</value>
		public Vector3 LinearVelocity{ get; private set; }

		/// <summary>
		/// Gets the actual angular velocity.
		/// </summary>
		/// <value>The angular velocity.</value>
		public Vector3 AngularVelocity{ get; private set; }

		/// <summary>
		/// Gets the actual rotation status quaternion.
		/// </summary>
		/// <value>The rotation status.</value>
		public Quaternion RotationStatus{ get; private set; }

		/// <summary>
		/// Gets the actual rotation matrix.
		/// </summary>
		/// <value>The rotation matrix.</value>
		public Matrix3x3 RotationMatrix{ get; private set; }

		#endregion

		#region Simulation Properties

		/// <summary>
		/// Gets a value indicating whether this <see cref="ObjectDefinition.SimulationObject"/> exclude from collision detection.
		/// </summary>
		/// <value><c>true</c> if exclude from collision detection; otherwise, <c>false</c>.</value>
		public bool ExcludeFromCollisionDetection{ get; private set; } 

		#endregion

		#region Constructor


		#endregion

		#region Public methods

		public void SetMass(double mass)
		{
			this.Mass = mass;
			this.InverseMass = 1.0 / this.Mass;
		}

		public void SetRestitutionCoeff(double restitutionCoeff)
		{
			this.RestitutionCoeff = restitutionCoeff;
		}

		public void SetStaticFrictionCoeff(double staticFrictionCoeff)
		{
			this.StaticFrictionCoeff = staticFrictionCoeff;
		}

		public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
		{
			this.DynamicFrictionCoeff = dynamicFrictionCoeff;
		}

		public void SetRelativePositions(Vector3[] inputRelativePositions)
		{
			Array.Copy (inputRelativePositions, this.RelativePositions, inputRelativePositions.Length);
		}

		public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
		{
			this.BaseInertiaTensor = Matrix3x3.Invert(inputIntertiaTensor);
		}

		public void SetInertiaTensor(Matrix3x3 inertiaTensor)
		{
			this.InertiaTensor = inertiaTensor;
		}

		public void SetPosition(Vector3 inputPosition)
		{
			this.Position = inputPosition;
		}

		public void SetStartPosition(Vector3 inputStartPosition)
		{
			this.StartPosition = new Vector3 (
				inputStartPosition.x, 
				inputStartPosition.y, 
				inputStartPosition.z);
		}

		public void SetLinearVelocity(Vector3 inputLinearVelocity)
		{
			this.LinearVelocity = inputLinearVelocity;
		}

		public void SetAngularVelocity(Vector3 inputAngularVelocity)
		{
			this.AngularVelocity = inputAngularVelocity;
		}

		public void SetRotationStatus(Quaternion inputRotationStatus)
		{
			this.RotationStatus = inputRotationStatus;
		}

		public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
		{
			this.RotationMatrix = inputRotationMatrix;
		}

		public void SetExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
		{
			this.ExcludeFromCollisionDetection = excludeFromCollisionDetection;
		}

		public void SetRelativePosition()
		{
			if (this.ObjectGeometry.VertexInitialPosition.Length > 0) 
			{
				this.RelativePositions = new Vector3[this.ObjectGeometry.VertexInitialPosition.Length];
				for (int i = 0; i < this.ObjectGeometry.VertexInitialPosition.Length; i++) 
					this.RelativePositions [i] = this.ObjectGeometry.VertexInitialPosition [i] - this.StartPosition;
			}
		}

		public void SetObjectType(ObjectType type)
		{
			this.ObjectType = type;
		}

		#endregion


	}
}

