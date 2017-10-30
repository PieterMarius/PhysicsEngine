using System;
using SharpEngineMathUtility;
using System.Collections.Generic;
using System.Linq;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public sealed class SoftShape : IShape, ISoftShape, Identity
    {
        #region Object status properties

        /// <summary>
        /// Shape ID
        /// </summary>
        int Identity.ID { get; set; }

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
        public ObjectType ObjectType { get { return ObjectType.SoftBody; } }

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
        public Vector3 Position { get; private set; }

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
        public Vector3 TempLinearVelocity { get; private set; }

        /// <summary>
        /// Gets the actual angular velocity.
        /// </summary>
        /// <value>The angular velocity.</value>
        public Vector3 AngularVelocity { get; private set; }

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

        public int GetID()
        {
            return ((Identity)this).ID;
        }

        /// <summary>
        /// Gets a value indicating whether this SimulationObject exclude from collision detection.
        /// </summary>
        /// <value><c>true</c> if exclude from collision detection; otherwise, <c>false</c>.</value>
        public bool ExcludeFromCollisionDetection { get; private set; }

        public AABB AABBox { get; private set; }

        public SoftShapePoint[] ShapePoints { get; private set; }

        public TriangleIndexes[] Triangle { get; private set; }

        public SoftPoint[] Sphere { get; private set; }

        public List<SoftBodyConstraint> SoftConstraint { get; private set; }

        public IShapeConvexDecomposition ConvexDecomposition { get; private set; }

        public double DecompositionParameter { get; private set; }

        #endregion

        #region Private Const

        private const double diameter = 0.1;

        #endregion

        #region Constructor

        public SoftShape(
            TriangleIndexes[] triangleIndex,
            SoftShapePoint[] shapePoint,
            List<SoftBodyConstraint> softConstraints)
        {
            Triangle = triangleIndex;
            ShapePoints = shapePoint;
            InertiaTensor = Matrix3x3.IdentityMatrix();
            SoftConstraint = softConstraints;
            SleepingFrameCount = 0;
            ConvexDecomposition = new ShapeConvexDecomposition(AABBox, Triangle);
        }

        public SoftShape(
            TriangleIndexes[] triangleIndex,
            Vector3[] shapePoint,
            Vector3 startPosition,
            double decompositionParam,
            double restoreCoefficient,
            double springCoefficient)
        {
            Mass = 1.0;
            Triangle = triangleIndex;

            DecompositionParameter = decompositionParam;
                                    
            InertiaTensor = Matrix3x3.IdentityMatrix();

            Position = startPosition;

            AddSoftShapePoint(shapePoint);
            BuildSoftConstraint(restoreCoefficient, springCoefficient);
                        
            SleepingFrameCount = 0;

            SetAABB();

            ConvexDecomposition = new ShapeConvexDecomposition(AABBox, Triangle);
        }

        #endregion

        #region Public Methods

        public void SetMass(double mass)
        {
            Mass = mass;
        }

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
            AABBox = AABB.GetShapePointAABB(ShapePoints);
        }

        public void SetPointsMass(double mass)
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

        public void SetShapePoint(SoftShapePoint[] shapePoint)
        {
            ShapePoints = shapePoint;
        }

        public void SetGeometrySphere(SoftPoint[] geometrySphere)
        {
            throw new NotImplementedException();
        }

        public void AddConstraint(SoftBodyConstraint constraint)
        {
            SoftConstraint.Add(constraint);
        }

        public void RemoveConstraint(int index)
        {
            SoftConstraint.RemoveAt(index);
        }

        public void SetDecompositionParameter(double decompositionParam)
        {
            DecompositionParameter = decompositionParam;
        }

        #endregion

        #region Private Methods

        private void AddSoftShapePoint(
            Vector3[] points)
        {
            ShapePoints = new SoftShapePoint[points.Length];

            double mass = Mass * (1.0 / points.Length);
            double inverseMass = 1.0 / mass;
            
            Matrix3x3 inertiaTensor = Matrix3x3.IdentityMatrix() *
                                     (diameter * diameter * 0.1 * mass);

            for (int i = 0; i < points.Length; i++)
            {
                ShapePoints[i] = new SoftShapePoint(diameter);
                ShapePoints[i].SetPosition(points[i] + Position);
                ShapePoints[i].SetLinearVelocity(LinearVelocity);
                ShapePoints[i].SetAngularVelocity(AngularVelocity);
                ShapePoints[i].SetMass(mass);
                ShapePoints[i].SetInverseMass(inverseMass);
                ShapePoints[i].SetBaseInertiaTensor(inertiaTensor);
                ShapePoints[i].SetInertiaTensor(Matrix3x3.InvertDiagonal(inertiaTensor));
                ShapePoints[i].SetRotationStatus(new Quaternion(1.0, 0.0, 0.0, 0.0));
            }
        }

        private void BuildSoftConstraint(
            double restoreCoefficient,
            double springCoefficient)
        {
            SoftConstraint = new List<SoftBodyConstraint>();
            HashSet<ConstraintIndex> indexHashSet = new HashSet<ConstraintIndex>();
            
            foreach (var triangle in Triangle.Select((value, i) => new { value, i }))
            {
                if(indexHashSet.Add(new ConstraintIndex(triangle.value.a, triangle.value.b)) &&
                   triangle.value.a != triangle.value.b)
                    SoftConstraint.Add(new SoftBodyConstraint(
                        ShapePoints[triangle.value.a],
                        ShapePoints[triangle.value.b],
                        this,
                        restoreCoefficient,
                        springCoefficient));

                if (indexHashSet.Add(new ConstraintIndex(triangle.value.a, triangle.value.c)) &&
                    triangle.value.a != triangle.value.c)
                    SoftConstraint.Add(new SoftBodyConstraint(
                        ShapePoints[triangle.value.a],
                        ShapePoints[triangle.value.c],
                        this,
                        restoreCoefficient,
                        springCoefficient));

                if (indexHashSet.Add(new ConstraintIndex(triangle.value.b, triangle.value.c)) &&
                    triangle.value.b != triangle.value.b)
                    SoftConstraint.Add(new SoftBodyConstraint(
                        ShapePoints[triangle.value.b],
                        ShapePoints[triangle.value.c],
                        this,
                        restoreCoefficient,
                        springCoefficient));

                //Add triangle index to shape points
                ShapePoints[triangle.value.a].AddTrianglesIndex(triangle.i);
                ShapePoints[triangle.value.b].AddTrianglesIndex(triangle.i);
                ShapePoints[triangle.value.c].AddTrianglesIndex(triangle.i);
            }
        }

        #endregion
    }
}
