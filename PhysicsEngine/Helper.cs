using System.Collections.Generic;
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
	}
}

