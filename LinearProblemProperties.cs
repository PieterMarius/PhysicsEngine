using System;

namespace GaussSeidel
{
	public class LinearProblemProperties
	{
		#region Properties

		//Matrice termini noti (N*N)
		public readonly double[] M;
		//Vettore dei valori attesi
		public readonly double[] B;
		//Diagonale matrice
        //TODO
		public double[] Diag { get; private set; }
		//Dimensioni vettori
		public readonly int Count;
		//Vettore delle incognite
        public double[] StartX { get; private set; } 

		#endregion

		#region Constructor

		public LinearProblemProperties (
			double[] M,
			double[] B,
			double[] startX,
			int count)
		{
			this.M = M;
			this.B = B;
            this.StartX = startX;
			this.Count = count;

            this.setDiag();
		}

		#endregion

        #region Public Methods

        public void SetStartValue(double[] X)
        {
            this.StartX = new double[this.Count];
            for (int i = 0; i < X.Length; i++)
            {
                this.StartX[i] = X[i];
            }
        }

        #endregion

        #region Private Methods

        private void setDiag()
        {
            this.Diag = new double[this.Count];
            for (int i = 0; i < this.Count; i++)
            {
                int index = i * this.Count;
                this.Diag[i] = 1.0 / M[index+i];
            }
        }

        #endregion
    }
}

