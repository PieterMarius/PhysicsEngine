using System;
using System.Collections.Generic;
using CollisionEngine;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public static class Helper
	{
		public static JacobianContact[] FindConstraints(
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

		public static JacobianContact[] FindConstraints(
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

		public static JacobianContact[] FindConstraints(
			JacobianContact[] list,
			ConstraintType typeA,
			ConstraintType typeB,
			ConstraintType typeC)
		{
			var result = new List<JacobianContact>();

			foreach (JacobianContact jc in list)
			{
				if (jc.Type == typeA ||
					jc.Type == typeB ||
				    jc.Type == typeC)
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

		public static JacobianContact[] PruneConstraintsFromFriction(
			JacobianContact[] list)
		{
			var result = new List<JacobianContact>();

			foreach (JacobianContact jc in list)
			{
				if (jc.Type != ConstraintType.Friction)
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
	}
}

