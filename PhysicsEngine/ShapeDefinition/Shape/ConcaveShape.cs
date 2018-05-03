using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal class ConcaveShape : Shape
    {
        #region Fields

        // <summary>
        /// Gets or sets the object geometry.
        /// </summary>
        /// <value>The object geometry.</value>
        public IGeometry[] ObjectGeometry { get; private set; }

        /// <summary>
        /// Axis Aligned Bounding Box
        /// </summary>
        public AABB AABBox { get; private set; }

        #endregion

        #region Constructor

        public ConcaveShape(
            TriangleMesh[] triangleMeshes,
            Vector3[] inputVertexPosition,
            Vector3 position,
            double mass,
            bool isStatic)
        {
            ObjectType = ObjectType.RigidBody;
            Mass = mass;

           // ShapeConvexDecomposition convexDecomposition = new ShapeConvexDecomposition(AABBox, softShape.Triangle);

        }

        #endregion

        #region Public Methods

        public override void Rotate(Vector3 versor, double angle)
        {
            throw new NotImplementedException();
        }

        public override void SetAABB()
        {
            //AABBox = AABB.GetShapePointAABB(ShapePoints);
        }

        public override void SetMass(double mass)
        {
            Mass = mass;
        }

        #endregion
    }
}
