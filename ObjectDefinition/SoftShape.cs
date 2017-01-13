using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
    public class SoftShape : IShape, ISoftShape
    {
        #region Object status properties

        /// <summary>
        /// Mass of object point.
        /// </summary>
        /// <value>The mass.</value>
        public double Mass { get; private set; }

        /// <summary>
        /// Gets the inverse mass point.
        /// </summary>
        /// <value>The inverse mass.</value>
        public double InverseMass { get; private set; }

        /// <summary>
        /// Elastict coefficient.
        /// </summary>
        /// <value>The restitution coeff.</value>
        public double RestitutionCoeff { get; private set; }

        /// <summary>
        /// Gets the static friction coeff.
        /// </summary>
        /// <value>The static friction coeff.</value>
        public double StaticFrictionCoeff { get; private set; }

        /// <summary>
        /// Gets the dynamic friction coeff.
        /// </summary>
        /// <value>The dynamic friction coeff.</value>
        public double DynamicFrictionCoeff { get; private set; }

        /// <summary>
        /// Gets the baumgarte stabilization coeff.
        /// </summary>
        /// <value>The baumgarte stabilization coeff.</value>
        public double RestoreCoeff { get; private set; }

        /// <summary>
        /// Gets the base inertia tensor ^(-1) of each point.
        /// </summary>
        /// <value>The inertia tensor.</value>
        public Matrix3x3 BaseInertiaTensor { get; private set; }

        /// <summary>
        /// Gets the inertia tensor ^(-1) of each point.
        /// </summary>
        /// <value>The inverse inertia tensor.</value>
        public Matrix3x3 InertiaTensor { get; private set; }

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
        public Vector3 Position
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        /// <summary>
        /// Gets the mass center start position.
        /// </summary>
        /// <value>The start position.</value>
        public Vector3 StartPosition { get; private set; }

        /// <summary>
        /// Gets the actual linear velocity.
        /// </summary>
        /// <value>The linear velocity.</value>
        public Vector3 LinearVelocity
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        /// <summary>
        /// Gets the temp linear velocity.
        /// </summary>
        /// <value>The temp linear velocity.</value>
        public Vector3 TempLinearVelocity { get; private set; }

        /// <summary>
        /// Gets the actual angular velocity.
        /// </summary>
        /// <value>The angular velocity.</value>
        public Vector3 AngularVelocity
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        /// <summary>
        /// Gets the temp angular velocity.
        /// </summary>
        /// <value>The temp angular velocity.</value>
        public Vector3 TempAngularVelocity { get; private set; }

        /// <summary>
        /// Gets the actual rotation status quaternion.
        /// </summary>
        /// <value>The rotation status.</value>
        public Quaternion RotationStatus { get; private set; }

        /// <summary>
        /// Gets the actual rotation matrix.
        /// </summary>
        /// <value>The rotation matrix.</value>
        public Matrix3x3 RotationMatrix { get; private set; }

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

        /// <summary>
        /// Gets a value indicating whether this SimulationObject exclude from collision detection.
        /// </summary>
        /// <value><c>true</c> if exclude from collision detection; otherwise, <c>false</c>.</value>
        public bool ExcludeFromCollisionDetection { get; private set; }

        public AABB AABBox { get; private set; }

        public SoftShapePoint[] ShapePoint { get; private set; }

        public int[][] Triangle { get; private set; }
       
        #endregion

        #region Constructor

        public SoftShape(
            int[][] triangleIndex)
        {
            //TODO: da modificare
            ObjectType = ObjectType.SoftBody;

            for (int i = 0; i < triangleIndex.Length; i++)
            {
                Triangle[i] = new int[3];
                Triangle[i][0] = triangleIndex[i][0];
                Triangle[i][1] = triangleIndex[i][1];
                Triangle[i][2] = triangleIndex[i][2];
            }

            InertiaTensor = Matrix3x3.IdentityMatrix();
            SleepingFrameCount = 0;
        }

        #endregion

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
            throw new NotImplementedException();
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            throw new NotImplementedException();
        }

        public void SetTempLinearVelocity(Vector3 inputLinearVelocity)
        {
            TempLinearVelocity = inputLinearVelocity;
        }

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            throw new NotImplementedException();
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
            AABBox = Helper.UpdateAABB(ShapePoint);
        }

        public void SetPointsMass(double mass)
        {
            Mass = mass;
            
            if (ObjectType == ObjectType.StaticRigidBody)
            {
                Mass = 0.0;
                InverseMass = 0.0;
            }
            else if (Mass > 0.0)
                InverseMass = 1.0 / Mass;
        }

        public void SetShapePoint(SoftShapePoint[] shapePoint)
        {
            ShapePoint = shapePoint;
        }
    }
}
