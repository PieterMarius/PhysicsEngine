using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpPhysicsEngine.Wrapper;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.NonConvexDecomposition
{
    public sealed class ConvexDecompositionEngine
    {
        #region Private Properties

        ShapeConvexDecomposition shapeConvexDecomposition;
        SoftShape baseSoftShape;

        #endregion

        #region Constructor

        public ConvexDecompositionEngine(SoftCollisionShape softShape)
        {
            baseSoftShape = (SoftShape)((IMapper)softShape).GetShape();
            shapeConvexDecomposition = new ShapeConvexDecomposition(baseSoftShape.AABBox, baseSoftShape.Triangle);
        }

        #endregion

        #region Public Methods

        public List<HashSet<Vertex3Index>> GetConvexShapeList(double precisionSize)
        {
            var vertex = Array.ConvertAll(baseSoftShape.ShapePoints, item => new Vertex3Index(
                                                                                item.Position,
                                                                                item.TriangleIndex.ToArray(), 0));

            return shapeConvexDecomposition.GetConvexShapeList(vertex, precisionSize).Select(x=>x.Vertex3Idx).ToList();
        }

        #endregion
    }
}
