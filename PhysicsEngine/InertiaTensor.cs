using System;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;

namespace MonoPhysicsEngine
{
	public class InertiaTensor
	{
		#region Private properties

		private Vector3 massCenter;
		private Matrix3x3 inertiaTensor;
		private double objMass;

		private Vector3[] vertexStartPosition;
		private int[][] triangleVertexIndex;

		#endregion

		#region Constructor

		public InertiaTensor (
			Vector3[] vertexStartPosition,
			int[][] triangleVertexIndex,
			double mass)
		{
			this.vertexStartPosition = vertexStartPosition;
			this.triangleVertexIndex = triangleVertexIndex;
			this.objMass = mass;

			this.massCenter = new Vector3 ();
			this.inertiaTensor = new Matrix3x3 (); 
			this.computeInertiaTensor ();
		}

		#endregion

		public Vector3 GetMassCenter()
		{
			return this.massCenter;
		}

		public Matrix3x3 GetInertiaTensor()
		{
			return this.inertiaTensor;
		}

		#region Public Methods

		#endregion

		#region Private Methods

		private void subExpression(
			ref double w0, 
			ref double w1, 
			ref double w2, 
			ref double f1, 
			ref double f2, 
			ref double f3, 
			ref double g0, 
			ref double g1, 
			ref double g2)
		{
			double temp0 = w0 + w1;
			f1 = temp0 + w2;
			double temp1 = w0 * w0;
			double temp2 = temp1 + w1 * temp0;
			f2 = temp2 + w2 * f1;
			f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
			g0 = f2 + w0 * (f1 + w0);
			g2 = f2 + w2 * (f1 + w2);
		}

		private void computeInertiaTensor()
		{
			double[] mult = new double[] {
				1.0 / 6.0,
				1.0 / 24.0,
				1.0 / 24.0,
				1.0 / 24.0,
				1.0 / 60.0,
				1.0 / 60.0,
				1.0 / 60.0,
				1.0 / 120.0,
				1.0 / 120.0,
				1.0 / 120.0
			};

			double[] intg = new double[]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			for (int i = 0; i < this.triangleVertexIndex.Length; i++) 
			{
				//Vertice 1 triangolo

				double x0 = this.vertexStartPosition [triangleVertexIndex [i] [0]].x;
				double y0 = this.vertexStartPosition [triangleVertexIndex [i] [0]].y;
				double z0 = this.vertexStartPosition [triangleVertexIndex [i] [0]].z;

				//Vertice 2 triangolo

				double x1 = this.vertexStartPosition [triangleVertexIndex [i] [1]].x;
				double y1 = this.vertexStartPosition [triangleVertexIndex [i] [1]].y;
				double z1 = this.vertexStartPosition [triangleVertexIndex [i] [1]].z;

				//Vertice 3 triangolo

				double x2 = this.vertexStartPosition [triangleVertexIndex [i] [2]].x;
				double y2 = this.vertexStartPosition [triangleVertexIndex [i] [2]].y;
				double z2 = this.vertexStartPosition [triangleVertexIndex [i] [2]].z;

				//Bordi e prodotto vettoriale 

				double a1 = x1 - x0;
				double b1 = y1 - y0;
				double c1 = z1 - z0;
				double a2 = x2 - x0;
				double b2 = y2 - y0;
				double c2 = z2 - z0;
				double d0 = b1 * c2 - b2 * c1;
				double d1 = a2 * c1 - a1 * c2;
				double d2 = a1 * b2 - a2 * b1;

				//Calcolo i termini dell'integrale

				double f1x = 0.0, f2x = 0.0, f3x = 0.0, g0x = 0.0, g1x = 0.0, g2x = 0.0;
				double f1y = 0.0, f2y = 0.0, f3y = 0.0, g0y = 0.0, g1y = 0.0, g2y = 0.0;
				double f1z = 0.0, f2z = 0.0, f3z = 0.0, g0z = 0.0, g1z = 0.0, g2z = 0.0;
				subExpression (ref x0, ref x1, ref x2, ref f1x, ref f2x, ref f3x, ref g0x, ref g1x, ref g2x);
				subExpression (ref y0, ref y1, ref y2, ref f1y, ref f2y, ref f3y, ref g0y, ref g1y, ref g2y);
				subExpression (ref z0, ref z1, ref z2, ref f1z, ref f2z, ref f3z, ref g0z, ref g1z, ref g2z);

				//Aggiorno l'integrale

				intg [0] += d0 * f1x;
				intg[1] += d0*f2x; intg[2] += d1*f2y; intg [3] += d2 * f2z;
				intg[4] += d0*f3x; intg[5] += d1*f3y; intg [6] += d2 * f3z;
				intg [7] += d0 * (y0 * g0x + y1 * g1x + y2 * g2x);
				intg [8] += d1 * (z0 * g0y + z1 * g1y + z2 * g2y);
				intg [9] += d2 * (x0 * g0z + x1 * g1z + x2 * g2z);
			}
			for (int i = 0; i < 10; i++) 
			{
				intg [i] *= mult [i];
			}
			double mass = intg [0];

			//centro di massa

			double massCenterX = intg [1] / mass;
			double massCenterY = intg [2] / mass;
			double massCenterZ = intg [3] / mass;

			this.massCenter = new Vector3 (massCenterX, massCenterY, massCenterZ);
			//matrice tensore d'inerzia sul centro di massa

			double r1x = intg [5] + intg [6] - mass * (this.massCenter.y * this.massCenter.y + this.massCenter.z * this.massCenter.z);
			double r2y = intg [4] + intg [6] - mass * (this.massCenter.z * this.massCenter.z + this.massCenter.x * this.massCenter.x);
			double r3z = intg [4] + intg [5] - mass * (this.massCenter.x * this.massCenter.x + this.massCenter.y * this.massCenter.y);
			double r1y = -(intg [7] - mass * this.massCenter.x * this.massCenter.y);
			double r2z = -(intg [8] - mass * this.massCenter.y * this.massCenter.z);
			double r1z = -(intg [9] - mass * this.massCenter.z * this.massCenter.x);
			double r2x = -(intg [7] - mass * this.massCenter.x * this.massCenter.y);
			double r3y = -(intg [8] - mass * this.massCenter.y * this.massCenter.z);
			double r3x = -(intg [9] - mass * this.massCenter.z * this.massCenter.x);
		
			this.inertiaTensor = new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);
			
			//L'oggetto ha massa totale 1, l'adatto alla massa richiesta
			double bufferMass = this.objMass / mass;

			this.inertiaTensor = this.inertiaTensor * bufferMass;
			
		}

		#endregion
	}
}

