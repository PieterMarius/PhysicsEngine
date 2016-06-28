using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace LCPSolver
{
	public class LinearProblemProperties
	{
		#region Properties

		//Matrice termini noti (N*N)
		public readonly SparseElement[] M;

		//Vettore dei valori attesi
		public readonly double[] B;

		//Diagonale matrice
		public readonly double[] D;

		//Vincoli
		public readonly double[] ConstraintLimit;

		//Tipo vincolo
		public readonly ConstraintType[] ConstraintType;

		//Legame contatto
		public readonly int?[] Constraints;

		//Dimensioni vettori
		public readonly int Count;

		//Vettore delle incognite
		public double[] StartX { get; private set; } 

		#endregion

		#region Constructor

		public LinearProblemProperties (
			SparseElement[] M,
			double[] B,
			double[] startX,
			double[] d,
			double[] constraintLimit,
			ConstraintType[] constraintType,
			int?[] constraints,
			int count)
		{
			this.M = M;
			this.B = B;
			StartX = startX;
			D = d;
			ConstraintLimit = constraintLimit;
			ConstraintType = constraintType;
			Constraints = constraints;
			Count = count;
		}

		#endregion

		#region Public Methods

		public void SetStartValue(double[] X)
		{
			Array.Copy (X, StartX, X.Length);
		}

		#endregion

	}
}

