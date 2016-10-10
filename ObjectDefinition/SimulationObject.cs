﻿using System;
using PhysicsEngineMathUtility;

namespace SimulationObjectDefinition
{
	public class SimulationObject
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
		/// Gets the baumgarte stabilization coeff.
		/// </summary>
		/// <value>The baumgarte stabilization coeff.</value>
		public double BaumgarteStabilizationCoeff{ get; private set; } 

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
		public ObjectGeometry ObjectGeometry{ get; private set; }

		/// <summary>
		/// Gets the type of the object.
		/// </summary>
		/// <value>The type of the object.</value>
		public ObjectType ObjectType { get; private set; }

        /// <summary>
        /// Get the geometry property of object
        /// </summary>
        public ObjectGeometryType ObjectGeometryType { get; private set; }
        
        /// <summary>
        /// Get the number of convex objects made the main object.
        /// </summary>
        public int CompoundingConvexObjectCount { get; private set; }

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
		/// Gets the temp linear velocity.
		/// </summary>
		/// <value>The temp linear velocity.</value>
		public Vector3 TempLinearVelocity{ get; private set; }

		/// <summary>
		/// Gets the actual angular velocity.
		/// </summary>
		/// <value>The angular velocity.</value>
		public Vector3 AngularVelocity{ get; private set; }

		/// <summary>
		/// Gets the temp angular velocity.
		/// </summary>
		/// <value>The temp angular velocity.</value>
		public Vector3 TempAngularVelocity{ get; private set; }

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

		/// <summary>
		/// Gets the force value.
		/// </summary>
		/// <value>The force value.</value>
		public Vector3 ForceValue { get; private set;}

		/// <summary>
		/// Gets the torque value.
		/// </summary>
		/// <value>The torque value.</value>
		public Vector3 TorqueValue { get; private set;}

		#endregion

		#region Simulation Properties

		/// <summary>
		/// Gets a value indicating whether this SimulationObject exclude from collision detection.
		/// </summary>
		/// <value><c>true</c> if exclude from collision detection; otherwise, <c>false</c>.</value>
		public bool ExcludeFromCollisionDetection{ get; private set; }

		#endregion

		#region Constructor

		public SimulationObject(
			ObjectType type,
			ObjectGeometry geometry,
			double mass,
			Vector3 position,
			Quaternion rotationStatus)
		{
			ObjectType = type;
			ObjectGeometry = geometry;
			Mass = mass;

			if (ObjectType == ObjectType.StaticRigidBody)
			{
				Mass = 0.0;
				InverseMass = 0.0;
			}
			else if (Mass > 0.0)
				InverseMass = 1.0 / Mass;

			RotationStatus = rotationStatus;

			SetObjectProperties();

            Position = position;

            ObjectGeometry.SetAABB(UpdateAABB(this));
        }

        #endregion

        #region Public methods

        public void SetRestitutionCoeff(double restitutionCoeff)
		{
			RestitutionCoeff = restitutionCoeff;
		}

		public void SetStaticFrictionCoeff(double staticFrictionCoeff)
		{
			StaticFrictionCoeff = staticFrictionCoeff;
		}

		public void SetDynamicFrictionCoeff(double dynamicFrictionCoeff)
		{
			DynamicFrictionCoeff = dynamicFrictionCoeff;
		}

		public void SetRelativePositions(Vector3[] inputRelativePositions)
		{
			Array.Copy (inputRelativePositions, RelativePositions, inputRelativePositions.Length);
		}

		public void SetBaseInertiaTensor(Matrix3x3 inputIntertiaTensor)
		{
			BaseInertiaTensor = Matrix3x3.Invert(inputIntertiaTensor);
		}

		public void SetInertiaTensor(Matrix3x3 inertiaTensor)
		{
			InertiaTensor = inertiaTensor;
		}

		public void SetPosition(Vector3 inputPosition)
		{
			Position = inputPosition;
		}

		public void SetStartPosition(Vector3 inputStartPosition)
		{
			StartPosition = new Vector3 (
				inputStartPosition.x, 
				inputStartPosition.y, 
				inputStartPosition.z);
		}

		public void SetLinearVelocity(Vector3 inputLinearVelocity)
		{
			LinearVelocity = inputLinearVelocity;
		}

		public void SetTempLinearVelocity(Vector3 inputLinearVelocity)
		{
			TempLinearVelocity = inputLinearVelocity;
		}

		public void SetAngularVelocity(Vector3 inputAngularVelocity)
		{
			AngularVelocity = inputAngularVelocity;
		}

		public void SetTempAngularVelocity(Vector3 inputAngularVelocity)
		{
			TempAngularVelocity = inputAngularVelocity;
		}

		public void SetRotationStatus(Quaternion inputRotationStatus)
		{
			RotationStatus = inputRotationStatus;
		}

		public void SetRotationMatrix(Matrix3x3 inputRotationMatrix)
		{
			RotationMatrix = inputRotationMatrix;
		}

		public void SetExcludeFromCollisionDetection(bool excludeFromCollisionDetection)
		{
			ExcludeFromCollisionDetection = excludeFromCollisionDetection;
		}

		public void SetTorque(Vector3 torque)
		{
			TorqueValue = torque;
		}

		public void SetForce(Vector3 force)
		{
			ForceValue = force;
		}

		public void SetBaumgarteStabilizationCoeff(double value)
		{
			BaumgarteStabilizationCoeff = value;
		}

        public void SetGeometryType(ObjectGeometryType geometryType)
        {
            ObjectGeometryType = geometryType;
        }

		#endregion

		#region Private Methods

		private void SetObjectProperties()
		{
            Vector3[] vertexPosition = Array.ConvertAll(ObjectGeometry.VertexPosition,
                                        item => item.Vertex);

            var inertiaTensor = new InertiaTensor(
					vertexPosition,
					ObjectGeometry.Triangle,
					Mass);

            var normalizedInertiaTensor = inertiaTensor;

            //Traslo per normalizzare l'oggetto rispetto al suo centro di massa
            if (inertiaTensor.GetMassCenter() != new Vector3())
            {
                for (int j = 0; j < ObjectGeometry.VertexPosition.Length; j++)
                {
                    ObjectGeometry.SetVertexPosition(
                        ObjectGeometry.VertexPosition[j].Vertex + inertiaTensor.GetMassCenter(),
                        j);
                }

                normalizedInertiaTensor = new InertiaTensor(
                    vertexPosition,
                    ObjectGeometry.Triangle,
                    Mass);
            }

            StartPosition = normalizedInertiaTensor.GetMassCenter();

            SetRelativePosition();

			RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));

			BaseInertiaTensor = Matrix3x3.Invert(inertiaTensor.GetInertiaTensor());
			InertiaTensor = (RotationMatrix * BaseInertiaTensor) * Matrix3x3.Transpose(RotationMatrix);
		}

		private void SetRelativePosition()
		{
			if (ObjectGeometry.VertexPosition.Length > 0)
			{
				RelativePositions = new Vector3[ObjectGeometry.VertexPosition.Length];
				for (int i = 0; i < ObjectGeometry.VertexPosition.Length; i++)
					RelativePositions[i] = ObjectGeometry.VertexPosition[i].Vertex - StartPosition;
			}
		}

        private AABB UpdateAABB(
            SimulationObject simObject)
        {
            Vector3 vertexPos = GetVertexPosition(simObject, 0);
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < simObject.RelativePositions.Length; i++)
            {
                Vector3 vertex = GetVertexPosition(simObject, i);

                if (vertex.x < xMin)
                    xMin = vertex.x;
                else if (vertex.x > xMax)
                    xMax = vertex.x;

                if (vertex.y < yMin)
                    yMin = vertex.y;
                else if (vertex.y > yMax)
                    yMax = vertex.y;

                if (vertex.z < zMin)
                    zMin = vertex.z;
                else if (vertex.z > zMax)
                    zMax = vertex.z;
            }

            return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
        }

        private Vector3 GetVertexPosition(
            SimulationObject obj,
            int index)
        {
            return
                obj.Position +
                (obj.RotationMatrix * obj.RelativePositions[index]);
        }

        #endregion
    }
}

