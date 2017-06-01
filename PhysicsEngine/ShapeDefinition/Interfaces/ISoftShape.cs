﻿
using SharpPhysicsEngine;
using System.Collections.Generic;

namespace SharpPhysicsEngine.ShapeDefinition
{
    public interface ISoftShape
    {
        SoftShapePoint[] ShapePoints { get; }
        TriangleIndexes[] Triangle { get; }
        AABB AABBox { get; }
        SoftPoint[] Sphere { get; }
        List<SoftBodyConstraint> SoftConstraint { get; }

        void SetPointsMass(double mass);
        void SetShapePoint(SoftShapePoint[] shapePoint);
        void SetAABB();
        void SetGeometrySphere(SoftPoint[] geometrySphere);
        void AddConstraint(SoftBodyConstraint constraint);
        void RemoveConstraint(int index);

    }
}
