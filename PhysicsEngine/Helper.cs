using System;
using System.Collections.Generic;
using CollisionEngine;
using SimulationObjectDefinition;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public static class Helper
	{
        #region Public Methods

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
            SimulationObject simObject)
        {
            Vector3 vertexPos = GetVertexPosition(simObject, 0);
            double xMax = vertexPos.x;
            double xMin = vertexPos.x;
            double yMax = vertexPos.y;
            double yMin = vertexPos.y;
            double zMax = vertexPos.z;
            double zMin = vertexPos.z;

            for (int i = 1; i < simObject.RelativePositions.Length; i++)
            {
                Vector3 vertex = GetVertexPosition(simObject, i);

                if (vertex.x < xMin)
                    xMin = vertex.x;
                else if (vertex.x > xMax)
                    xMax = vertex.x;

                if (vertex.y < yMin)
                    yMin = vertex.y;
                else if (vertex.y > yMax)
                    yMax = vertex.y;

                if (vertex.z < zMin)
                    zMin = vertex.z;
                else if (vertex.z > zMax)
                    zMax = vertex.z;
            }

            return new AABB(xMin, xMax, yMin, yMax, zMin, zMax, false);
        }

        public static Vector3 GetVertexPosition(
            SimulationObject obj,
            int index)
        {
            return
                obj.Position +
                (obj.RotationMatrix * obj.RelativePositions[index]);
        }

        #endregion
    }
}

