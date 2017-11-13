using System;
using SharpEngineMathUtility;

namespace SharpPhysicsEngine.ShapeDefinition
{
	public sealed class ConvexShape : IShape, IShapeCommon, IConvexShape, Identity
    {

        #region Object status properties

        /// <summary>
        /// Shape ID
        /// </summary>
        public int ID { get; private set; }

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
		public double RestoreCoeff{ get; private set; } 

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
		public IGeometry ObjectGeometry{ get; private set; }

		/// <summary>
		/// Gets the type of the object.
		/// </summary>
		/// <value>The type of the object.</value>
		public ObjectType ObjectType { get; private set; }
        
        /// <summary>
        /// Sleeping Frame Count value
        /// </summary>
        public int SleepingFrameCount { get; private set; }
        
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
		public Vector3 StartPosition { get; private set; }

        /// <summary>
        /// Gets the actual linear velocity.
        /// </summary>
        /// <value>The linear velocity.</value>
        public Vector3 LinearVelocity { get; private set; }

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
		public Vector3 ForceValue { get; private set; }

		/// <summary>
		/// Gets the torque value.
		/// </summary>
		/// <value>The torque value.</value>
		public Vector3 TorqueValue { get; private set; }

        #endregion

        #region Simulation Properties

        public void SetID(int id)
        {
            ID = id;
        }

        /// <summary>
        /// Gets a value indicating whether this SimulationObject exclude from collision detection.
        /// </summary>
        /// <value><c>true</c> if exclude from collision detection; otherwise, <c>false</c>.</value>
        public bool ExcludeFromCollisionDetection{ get; private set; }

		#endregion

		#region Constructor

        public ConvexShape(ObjectType type)
        {
            ObjectType = type;

            InertiaTensor = Matrix3x3.IdentityMatrix();
            SleepingFrameCount = 0;
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

		public void SetRestoreCoeff(double value)
		{
			RestoreCoeff = value;
		}
        
        public void SetSleepingFrameCount(int frameCount)
        {
            SleepingFrameCount = frameCount;
        }
                
        public void SetAABB()
        {
            if (ObjectGeometry != null)
                ObjectGeometry.SetAABB(AABB.GetGeometryAABB(ObjectGeometry));
        }

        public void SetMass(double mass)
        {
            Mass = mass;
            
            if (ObjectType == ObjectType.StaticBody)
            {
                Mass = 0.0;
                InverseMass = 0.0;
            }
            else if (Mass > 0.0)
                InverseMass = 1.0 / Mass;
        }

        public void SetObjectGeometry(IGeometry geometry)
        {
            ObjectGeometry = geometry;
            
            SetObjectProperties();
            SetAABB();
        }
        
        #endregion

        #region Private Methods
        
        private void SetObjectProperties()
        {
            Matrix3x3 baseTensors = new Matrix3x3();

            int totalVertex = 0;

            StartPosition = CalculateMassCenter();
                        
            ////TODO Mass check
            Vector3[] vertexPosition = Array.ConvertAll(
                                    ObjectGeometry.VertexPosition,
                                    item => item.Vertex);

            var inertiaTensor = new InertiaTensor(
                    vertexPosition,
                    ObjectGeometry.Triangle,
                    Mass,
                    true);

            var normalizedInertiaTensor = inertiaTensor;

            totalVertex += ObjectGeometry.VertexPosition.Length;

            Vector3 r = inertiaTensor.GetMassCenter() - StartPosition;
            baseTensors += inertiaTensor.GetInertiaTensor() +
                            (Matrix3x3.IdentityMatrix() * r.Dot(r) - Matrix3x3.OuterProduct(r, r)) *
                            Mass;
            
            RotationMatrix = Quaternion.ConvertToMatrix(Quaternion.Normalize(RotationStatus));

            SetRelativePosition(totalVertex);

            BaseInertiaTensor = Matrix3x3.Invert(baseTensors);
            InertiaTensor = (RotationMatrix * BaseInertiaTensor) *
                            Matrix3x3.Transpose(RotationMatrix);
        }

        private void SetRelativePosition(int totalVertex)
        {
            Vector3[] relativePositions = new Vector3[ObjectGeometry.VertexPosition.Length];
            if (ObjectGeometry.VertexPosition.Length > 0)
            {
                for (int j = 0; j < ObjectGeometry.VertexPosition.Length; j++)
                    relativePositions[j] =
                        ObjectGeometry.VertexPosition[j].Vertex -
                        StartPosition;
            }

            ObjectGeometry.SetRelativePosition(relativePositions);
        }

        private Vector3 CalculateMassCenter()
        {
            Vector3[] vertexPosition = Array.ConvertAll(
                                        ObjectGeometry.VertexPosition,
                                        item => item.Vertex);

            var inertiaTensor = new InertiaTensor(
                    vertexPosition,
                    ObjectGeometry.Triangle,
                    Mass,
                    false);

            return inertiaTensor.GetMassCenter();
        }

        #endregion
    }
}

