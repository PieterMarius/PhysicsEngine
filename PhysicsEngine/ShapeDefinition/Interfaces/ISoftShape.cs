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
        void SetGeometrySphere(SoftPoint[] geometrySphere);
        void AddConstraint(SoftConstraint constraint);
        void RemoveConstraint(int index);
        void SetDecompositionParameter(double decompositionParam);
        void SetRestoreCoefficient(double restoreCoeff);
        void SetSpringCoefficient(double springCoeff);
        
    }
}
