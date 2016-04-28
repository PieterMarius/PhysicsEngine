﻿using System;
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

				case ConstraintType.Friction:
					
					return GeometryUtilities.Clamp (
						X [i],
						X [i + input.Constraints [i].Value] * input.ConstraintLimitMin [i],
						-X [i + input.Constraints [i].Value] * input.ConstraintLimitMin [i]);
					
				case ConstraintType.Joint:
					return X [i];

				case ConstraintType.JointLimit:
					return Math.Max (0.0, X [i]);
				
				default:
					return X [i];
			}
		}
	}
}

