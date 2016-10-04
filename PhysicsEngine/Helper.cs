using System;
using System.Collections.Generic;
using CollisionEngine;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public static class Helper
	{
		public static JacobianContact[] FilterConstraints(
			JacobianContact[] list,
			ConstraintType typeA)
		{
			var result = new List<JacobianContact>();

			foreach (JacobianContact jc in list)
			{
				if (jc.Type == typeA)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianContact[] FilterConstraints(
			JacobianContact[] list,
			ConstraintType typeA,
			ConstraintType typeB)
		{
			var result = new List<JacobianContact>();

			foreach (JacobianContact jc in list)
			{
				if (jc.Type == typeA ||
					jc.Type == typeB)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianContact[] FindJointConstraints(JacobianContact[] list)
		{
			var result = new List<JacobianContact>();

			foreach (JacobianContact jc in list)
			{
				if (jc.Type != ConstraintType.Friction &&
					jc.Type != ConstraintType.Collision)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianContact[] FindConstraintsWithError(
			JacobianContact[] list,
			ConstraintType typeA,
			ConstraintType typeB)
		{
			var result = new List<JacobianContact>();

			foreach (JacobianContact jc in list)
			{
				if ((jc.Type == typeA ||
				    jc.Type == typeB) &&
				    Math.Abs(jc.CorrectionValue) > 1E-100)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianContact[] PruneConstraintsFromSoftJoint(
			JacobianContact[] list)
		{
			var result = new List<JacobianContact>();

			foreach (JacobianContact jc in list)
			{
				if (jc.Type != ConstraintType.SoftJoint)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static CollisionPointStructure Find(
			CollisionPointStructure[] collisionPoints,
			ContactIndex contactIndex)
		{
			foreach (CollisionPointStructure cps in collisionPoints)
			{
				if (cps.ObjectA == contactIndex.IndexA &&
				    cps.ObjectB == contactIndex.IndexB)
				{
					return cps;
				}
			}
			return null;
		}

        public static AABB UpdateAABB(
            ObjectGeometry objectGeometry)
        {
            double xMax = objectGeometry.VertexPosition[0].Vertex.x;
            double xMin = objectGeometry.VertexPosition[0].Vertex.x;
            double yMax = objectGeometry.VertexPosition[0].Vertex.y;
            double yMin = objectGeometry.VertexPosition[0].Vertex.y;
            double zMax = objectGeometry.VertexPosition[0].Vertex.z;
            double zMin = objectGeometry.VertexPosition[0].Vertex.z;

            for (int i = 1; i < objectGeometry.VertexPosition.Length; i++)
            {
                Vector3 vertex = objectGeometry.VertexPosition[i].Vertex;

                if (vertex.x < xMin)
                    xMin = vertex.x;

                if (vertex.x > xMax)
                    xMax = vertex.x;

                if (vertex.y < yMin)
                    yMin = vertex.y;

                if (vertex.y > yMax)
                    yMax = vertex.y;

                if (vertex.z < zMin)
                    zMin = vertex.z;

                if (vertex.z > zMax)
                    zMax = vertex.z;
            }

            return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
        }
    }
}

