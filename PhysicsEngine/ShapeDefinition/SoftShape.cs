using System;
using SharpEngineMathUtility;
using System.Collections.Generic;
using System.Linq;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class SoftShape : IShape, ISoftShape, IDentity
    {
        #region Object status properties

        /// <summary>
        /// Shape ID
        /// </summary>
        public int ID { get; private set; }

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

        public void SetID(int id)
        {
            ID = id;
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

        public List<SoftConstraint> SoftConstraint { get; private set; }

        public IShapeConvexDecomposition ConvexDecomposition { get; private set; }

        public double DecompositionParameter { get; private set; }

        #endregion

        #region Private Const

        private const double diameter = 0.01;

        #endregion

        #region Constructor

        public SoftShape(
            TriangleIndexes[] triangleIndex,
            Vector3[] shapePoint,
            ConstraintIndex[] softConstraints,
            double decompositionParam,
            double restoreCoeff,
            double springCoeff)
        {
            Triangle = triangleIndex;
            InertiaTensor = Matrix3x3.IdentityMatrix();
            AddSoftShapePoint(shapePoint);
            BuildSoftConstraints(softConstraints, restoreCoeff, springCoeff);
            SleepingFrameCount = 0;
            ConvexDecomposition = new ShapeConvexDecomposition(AABBox, Triangle);
        }

        public SoftShape(
            TriangleIndexes[] triangleIndex,
            Vector3[] shapePoint,
            Vector3 startPosition,
            double decompositionParam,
            double restoreCoeff,
            double springCoeff)
        {
            Mass = 1.0;
            Triangle = triangleIndex;

            DecompositionParameter = decompositionParam;
                                    
            InertiaTensor = Matrix3x3.IdentityMatrix();

            Position = startPosition;

            AddSoftShapePoint(shapePoint);
            BuildSoftConstraints(restoreCoeff, springCoeff);
                        
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
            throw new NotSupportedException();
        }

        public void SetLinearVelocity(Vector3 inputLinearVelocity)
        {
            throw new NotSupportedException();
        }

        public void SetTempLinearVelocity(Vector3 inputLinearVelocity)
        {
            TempLinearVelocity = inputLinearVelocity;
        }

        public void SetAngularVelocity(Vector3 inputAngularVelocity)
        {
            throw new NotSupportedException();
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
        
        public void SetConstraintsRestoreCoefficient(double restoreCoeff)
        {
            foreach (var item in SoftConstraint)
                item.SetRestoreCoefficient(restoreCoeff);
        }

        public void SetConstraintsSpringCoefficient(double springCoeff)
        {
            foreach (var item in SoftConstraint)
                item.SetSpringCoefficient(springCoeff);
        }

        public void AddConstraint(SoftConstraint constraint)
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

        public void Rotate(Vector3 versor, double angle)
        {
            throw new NotImplementedException();
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
                                    (diameter * diameter * 0.4);

            for (int i = 0; i < points.Length; i++)
            {
                ShapePoints[i] = new SoftShapePoint(diameter);
                ShapePoints[i].SetStartPosition(points[i]);
                ShapePoints[i].SetPosition(points[i] + Position);

                Vector3 r = -1.0 * ShapePoints[i].StartPosition;
                Matrix3x3 baseTensors = Matrix3x3.Invert(
                                inertiaTensor +
                                (Matrix3x3.IdentityMatrix() * r.Dot(r) - Matrix3x3.OuterProduct(r, r)) *
                                mass);

                ShapePoints[i].SetLinearVelocity(LinearVelocity);
                ShapePoints[i].SetAngularVelocity(AngularVelocity);
                ShapePoints[i].SetMass(mass);
                ShapePoints[i].SetInverseMass(inverseMass);
                ShapePoints[i].SetBaseInertiaTensor(baseTensors);
                ShapePoints[i].SetInertiaTensor(baseTensors);
                ShapePoints[i].SetRotationStatus(new Quaternion(1.0, 0.0, 0.0, 0.0));
                ShapePoints[i].SetRotationMatrix(Quaternion.ConvertToMatrix(RotationStatus));
            }
        }

        private void BuildSoftConstraints(
            ConstraintIndex[] constraintIndex,
            double restoreCoeff,
            double springCoeff)
        {
            SoftConstraint = new List<SoftConstraint>();
            HashSet<ConstraintIndex> indexHashSet = new HashSet<ConstraintIndex>();

            for (int i = 0; i < constraintIndex.Length; i++)
            {
                if (indexHashSet.Add(constraintIndex[i]))
                {
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[constraintIndex[i].IndexA],
                        ShapePoints[constraintIndex[i].IndexB],
                        this,
                        restoreCoeff,
                        springCoeff));
                }
            }
        }

        private void BuildSoftConstraints(
            double restoreCoeff,
            double springCoeff)
        {
            SoftConstraint = new List<SoftConstraint>();
            HashSet<ConstraintIndex> indexHashSet = new HashSet<ConstraintIndex>();
            
            foreach (var triangle in Triangle.Select((value, i) => new { value, i }))
            {
                if(indexHashSet.Add(new ConstraintIndex(triangle.value.a, triangle.value.b)) &&
                   triangle.value.a != triangle.value.b)
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[triangle.value.a],
                        ShapePoints[triangle.value.b],
                        this,
                        restoreCoeff,
                        springCoeff));

                if (indexHashSet.Add(new ConstraintIndex(triangle.value.a, triangle.value.c)) &&
                    triangle.value.a != triangle.value.c)
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[triangle.value.a],
                        ShapePoints[triangle.value.c],
                        this,
                        restoreCoeff,
                        springCoeff));

                if (indexHashSet.Add(new ConstraintIndex(triangle.value.b, triangle.value.c)) &&
                    triangle.value.b != triangle.value.c)
                    SoftConstraint.Add(new SoftConstraint(
                        ShapePoints[triangle.value.b],
                        ShapePoints[triangle.value.c],
                        this,
                        restoreCoeff,
                        springCoeff));

                //Add triangle index to shape points
                ShapePoints[triangle.value.a].AddTrianglesIndex(triangle.i);
                ShapePoints[triangle.value.b].AddTrianglesIndex(triangle.i);
                ShapePoints[triangle.value.c].AddTrianglesIndex(triangle.i);
            }
        }

        #endregion
    }
}
