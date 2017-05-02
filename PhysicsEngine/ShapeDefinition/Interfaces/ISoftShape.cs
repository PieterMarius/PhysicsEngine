
using SharpPhysicsEngine;
using System.Collections.Generic;

namespace ShapeDefinition
{
    public interface ISoftShape
    {
        SoftShapePoint[] ShapePoints { get; }
        int[][] Triangle { get; }
        AABB AABBox { get; }
        SoftPoint[] Sphere { get; }
        List<JacobianConstraint> SoftConstraint { get; }

        void SetPointsMass(double mass);
        void SetShapePoint(SoftShapePoint[] shapePoint);
        void SetAABB();
        void SetGeometrySphere(SoftPoint[] geometrySphere);
        void AddConstraint(JacobianConstraint constraint);
        void RemoveConstraint(int index);

    }
}
