using System;
using SharpEngineMathUtility;

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

        }

        #endregion

        #region Public Methods

        public override void Rotate(Vector3 versor, double angle)
        {
            throw new NotImplementedException();
        }

        public override void SetAABB()
        {
            throw new NotImplementedException();
        }

        public override void SetMass(double mass)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
