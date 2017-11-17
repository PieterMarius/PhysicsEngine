using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using System.Collections.Generic;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal interface ISoftShape
    {
        SoftShapePoint[] ShapePoints { get; }
        TriangleIndexes[] Triangle { get; }
        AABB AABBox { get; }
        SoftPoint[] Sphere { get; }
        List<SoftConstraint> SoftConstraint { get; }
        IShapeConvexDecomposition ConvexDecomposition { get; }
        double DecompositionParameter { get; }
        
        void SetPointsMass(double mass);
        void SetShapePoint(SoftShapePoint[] shapePoint);
        void SetAABB();
        void AddConstraint(SoftConstraint constraint);
        void RemoveConstraint(int index);
        void SetDecompositionParameter(double decompositionParam);
        void SetConstraintsRestoreCoefficient(double restoreCoeff);
        void SetConstraintsSpringCoefficient(double springCoeff);
    }
}
