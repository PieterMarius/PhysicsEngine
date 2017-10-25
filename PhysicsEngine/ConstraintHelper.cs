using System;
using System.Collections.Generic;
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
	public static class ConstraintHelper
	{
        #region Public Methods

        public static JacobianConstraint[] FilterConstraints(
			JacobianConstraint[] list,
			ConstraintType typeA)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if (jc.Type == typeA)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] FilterConstraints(
			JacobianConstraint[] list,
			ConstraintType typeA,
			ConstraintType typeB)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if (jc.Type == typeA ||
					jc.Type == typeB)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] FindJointConstraints(JacobianConstraint[] list)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if (jc.Type != ConstraintType.Friction &&
					jc.Type != ConstraintType.Collision)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] FindConstraintsWithError(
			JacobianConstraint[] list,
			ConstraintType typeA,
			ConstraintType typeB)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if ((jc.Type == typeA ||
				    jc.Type == typeB) &&
				    Math.Abs(jc.CorrectionValue) > 1E-100)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] PruneConstraintsFromSoftJoint(
			JacobianConstraint[] list)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
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
				if (cps.ObjectIndexA == contactIndex.IndexA &&
				    cps.ObjectIndexB == contactIndex.IndexB)
				{
					return cps;
				}
			}
			return null;
		}
        
        #endregion
    }
}

