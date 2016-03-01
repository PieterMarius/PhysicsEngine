using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace LCPSolver
{
	public static class ClampSolution
	{
		public static double ClampX(
			LinearProblemProperties input,
			double[] X,
			int i)
		{

			switch (input.ConstraintType[i]) 
			{
			case ConstraintType.Collision:
				return Math.Max (0.0, X [i]);

			case ConstraintType.StaticFriction:
				return GeometryUtilities.Clamp (
					X [i],
					X [i + input.Constraints [i].Value] * input.ConstraintLimit [i],
					-X [i + input.Constraints [i].Value] * input.ConstraintLimit [i]);
				
			case ConstraintType.DynamicFriction:
				return GeometryUtilities.Clamp (
					X [i],
					X [i + input.Constraints [i].Value] * input.ConstraintLimit [i],
					-X [i + input.Constraints [i].Value] * input.ConstraintLimit [i]);
				
			case ConstraintType.Joint:
				return X [i];
			
			default:
				return X [i];
			}
		}
	}
}

