using System;
using PhysicsEngineMathUtility;

namespace LCPSolver
{
	public class LCPSolver: ISolver
	{
		#region Properties

		private SolverParameters solverParameters;

		#endregion

		#region Constructor

		public LCPSolver (
			SolverParameters solverParameters)
		{
			this.solverParameters = solverParameters;
		}

		#endregion

		#region Public Methods

		public void Solve(LinearProblemProperties linearProblemProperties)
		{
			if (linearProblemProperties.Count > 0) {

				double internalSOR = this.solverParameters.SOR;
				double moduleOnlineGradient = 0.0;
				double modOGa = 0.0;
				double bk = 0.0;
				double test = 0.0;
				double[] oldX = new double[linearProblemProperties.Count];
				double[] deltaf = new double[linearProblemProperties.Count];
				double[] pk = new double[linearProblemProperties.Count];
				double[] buffer = new double[linearProblemProperties.Count];

				Array.Copy (linearProblemProperties.StartX, oldX, linearProblemProperties.Count);

				this.gaussSeidel (
					linearProblemProperties, 
					this.solverParameters.SOR,
					ref buffer);

				deltaf = this.calculateGradient (oldX, linearProblemProperties.StartX);
				this.copyNegArray (ref pk, deltaf);

				for (int i = 0; i < this.solverParameters.MaxIteration; i++) {
					Array.Copy (linearProblemProperties.StartX, oldX, linearProblemProperties.Count);
					moduleOnlineGradient = this.arrayModule (deltaf);

					if (moduleOnlineGradient < this.solverParameters.ErrorTolerance)
						break;
					if (i == 0)
						modOGa = moduleOnlineGradient;
					else {
						test = modOGa - moduleOnlineGradient;
						if (test < 0.0 && internalSOR > 0.0)
							internalSOR -= this.solverParameters.SORStep;
						modOGa = moduleOnlineGradient;
					}

					this.gaussSeidel (
						linearProblemProperties,  
						internalSOR,
						ref buffer);

					deltaf = this.calculateGradient (oldX, linearProblemProperties.StartX);

					bk = 0.0;

					if (moduleOnlineGradient != 0.0)
						bk = this.arrayModule (deltaf) / moduleOnlineGradient;
					else {
						bk = 0.0;
						continue;
					}


					if (bk <= 1.0) {
						for (int j = 0; j < linearProblemProperties.Count; j++) {
							linearProblemProperties.StartX [j] += bk * pk [j];
							buffer [j] = oldX [j];
							pk [j] = bk * pk [j] - deltaf [j];
							this.clampX (linearProblemProperties, j);
						}	
					}
				}
			}
		}

		#region Solver Parameters 

		public void SetSolverParameters(SolverParameters solverParameters)
		{
			this.solverParameters = solverParameters;
		}
			
		public SolverParameters GetSolverParameters()
		{
			return this.solverParameters;
		}

		#endregion


		#endregion

		#region Private Methods

		private void gaussSeidel(
			LinearProblemProperties input,
			double internalSOR,
			ref double[] buffer)
		{
			double[] sum = new double[input.Count];

			this.lowerTriangularMatrix (input, ref sum);
			for (int i = 0; i < input.Count; i++) 
			{
				this.internalIteration (input, ref sum, ref buffer, internalSOR, i);
				this.clampX (input,i);
				buffer [i] = input.X [i];
			}
		}

		private void clampX(
			LinearProblemProperties input,
			int i)
		{
			switch (input.ConstraintType[i]) 
			{
				case ConstraintType.Collision:
					input.X [i] = Math.Max (0.0, input.X [i]);
					break;
				case ConstraintType.StaticFriction:
					input.X [i] = GeometryUtilities.Clamp (
						input.X [i],
						input.X [i + input.Constraints[i]] * input.ConstraintLimit [i],
						-input.X [i + input.Constraints[i]] * input.ConstraintLimit [i]);
					break;
				case ConstraintType.DynamicFriction:
					input.X [i] = GeometryUtilities.Clamp (
						input.X [i],
						input.X [i + input.Constraints[i]] * input.ConstraintLimit [i],
						-input.X [i + input.Constraints[i]] * input.ConstraintLimit [i]);
					break;
				case ConstraintType.Joint:
					break;
				default:
					break;
			}
		}

		private void internalIteration(
			LinearProblemProperties input,
			ref double[] sum,
			ref double[] buffer,
			double internalSOR,
			int i)
		{
			int index = i * input.Count;
			double sumBuffer = 0.0;

			for (int j = 0; j < i; j++)
				sumBuffer += input.M [index + j] * input.X [j];
			
			sum [i] += sumBuffer;
			sum [i] = (input.B [i] - sum [i]) * input.Diag [i];
			input.X [i] = buffer [i] + (sum [i] - buffer [i]) * internalSOR;
		}

		private void lowerTriangularMatrix(
			LinearProblemProperties input,
			ref double[] sum)
		{
			for (int i = 0; i < input.Count; i++)
				sum[i] = this.kernel (input, i);
		}

		private double kernel(
			LinearProblemProperties input,
			int i)
		{
			double sumBuffer = 0.0;
			int index = i * input.X.Length;
			for (int j = 0; j < input.Count; j++)
				sumBuffer += input.M [index + j] * input.X [j];
			return sumBuffer;
		}

		private void copyNegArray(
			ref double[] destination,
			double[] source)
		{
			for (int i = 0; i < source.Length; i++)
				destination [i] = - source [i];
		}

		private double arrayModule(
			double[] array)
		{
			double module = 0.0;
			for (int i = 0; i < array.Length; i++) 
			{
				module += array [i] * array [i];
			}
			return module;
		}

		private double[] calculateGradient(
			double[] x0,
			double[] x1)
		{
			double[] gradient = new double[x0.Length];
			for (int i = 0; i < x0.Length; i++)
				gradient [i] = -(x1 [i] - x0 [i]);

			return gradient;
		}

		#endregion
	}
}

