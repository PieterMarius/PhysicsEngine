using System;
using System.Collections.Generic;
using CollisionEngine;
using ShapeDefinition;
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
        
        #endregion
    }
}

