using PhysicsEngineMathUtility;

namespace ShapeDefinition
{
	public class InertiaTensor
	{
		#region Private properties

		Vector3 massCenter;
		Matrix3x3 inertiaTensor;
        bool bodyCoords;
		readonly double objMass;

		readonly Vector3[] vertexStartPosition;
		readonly int[][] triangleVertexIndex;
                
		readonly double[] mult =
		{
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

		readonly double[] intg = 
		{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };


		#endregion

		#region Constructor

		public InertiaTensor (
			Vector3[] vertexStartPosition,
			int[][] triangleVertexIndex,
			double mass,
            bool bodyCoords = true)
		{
			this.vertexStartPosition = vertexStartPosition;
			this.triangleVertexIndex = triangleVertexIndex;
            this.bodyCoords = bodyCoords;
			objMass = mass;

			massCenter = new Vector3 ();
			inertiaTensor = new Matrix3x3 ();
                         
			computeInertiaTensor ();
		}

		#endregion

		public Vector3 GetMassCenter()
		{
			return massCenter;
		}

		public Matrix3x3 GetInertiaTensor()
		{
			return inertiaTensor;
		}

        #region Public Methods

        #endregion

        #region Private Methods

        private void computeInertiaTensor()
		{
			
			for (int i = 0; i < triangleVertexIndex.Length; i++) 
			{
                //Vertice 1 triangolo
                Vector3 v0 = vertexStartPosition[triangleVertexIndex[i][0]];

                //Vertice 2 triangolo
                Vector3 v1 = vertexStartPosition[triangleVertexIndex[i][1]];

                //Vertice 3 triangolo
                Vector3 v2 = vertexStartPosition[triangleVertexIndex[i][2]];

                // Get cross product of edges and normal vector.
                Vector3 V1mV0 = v1 - v0;
                Vector3 V2mV0 = v2 - v0;
                Vector3 N = Vector3.Cross(V1mV0, V2mV0);

                // Compute integral terms.
                double tmp0, tmp1, tmp2;
                double f1x, f2x, f3x, g0x, g1x, g2x;
                tmp0 = v0[0] + v1[0];
                f1x = tmp0 + v2[0];
                tmp1 = v0[0] * v0[0];
                tmp2 = tmp1 + v1[0] * tmp0;
                f2x = tmp2 + v2[0] * f1x;
                f3x = v0[0] * tmp1 + v1[0] * tmp2 + v2[0] * f2x;
                g0x = f2x + v0[0] * (f1x + v0[0]);
                g1x = f2x + v1[0] * (f1x + v1[0]);
                g2x = f2x + v2[0] * (f1x + v2[0]);

                double f1y, f2y, f3y, g0y, g1y, g2y;
                tmp0 = v0[1] + v1[1];
                f1y = tmp0 + v2[1];
                tmp1 = v0[1] * v0[1];
                tmp2 = tmp1 + v1[1] * tmp0;
                f2y = tmp2 + v2[1] * f1y;
                f3y = v0[1] * tmp1 + v1[1] * tmp2 + v2[1] * f2y;
                g0y = f2y + v0[1] * (f1y + v0[1]);
                g1y = f2y + v1[1] * (f1y + v1[1]);
                g2y = f2y + v2[1] * (f1y + v2[1]);

                double f1z, f2z, f3z, g0z, g1z, g2z;
                tmp0 = v0[2] + v1[2];
                f1z = tmp0 + v2[2];
                tmp1 = v0[2] * v0[2];
                tmp2 = tmp1 + v1[2] * tmp0;
                f2z = tmp2 + v2[2] * f1z;
                f3z = v0[2] * tmp1 + v1[2] * tmp2 + v2[2] * f2z;
                g0z = f2z + v0[2] * (f1z + v0[2]);
                g1z = f2z + v1[2] * (f1z + v1[2]);
                g2z = f2z + v2[2] * (f1z + v2[2]);

                // Update integrals.
                intg[0] += N[0] * f1x;
                intg[1] += N[0] * f2x;
                intg[2] += N[1] * f2y;
                intg[3] += N[2] * f2z;
                intg[4] += N[0] * f3x;
                intg[5] += N[1] * f3y;
                intg[6] += N[2] * f3z;
                intg[7] += N[0] * (v0[1] * g0x + v1[1] * g1x + v2[1] * g2x);
                intg[8] += N[1] * (v0[2] * g0y + v1[2] * g1y + v2[2] * g2y);
                intg[9] += N[2] * (v0[0] * g0z + v1[0] * g1z + v2[0] * g2z);
            }

			for (int i = 0; i < 10; i++) 
			{
                intg[i] *= mult[i];
			}
			double mass = intg [0];

			//centro di massa

			double massCenterX = intg [1] / mass;
			double massCenterY = intg [2] / mass;
			double massCenterZ = intg [3] / mass;

			massCenter = new Vector3 (massCenterX, massCenterY, massCenterZ);

            double r1x = intg[5] + intg[6];
            double r2y = intg[4] + intg[6];
            double r3z = intg[4] + intg[5];
            double r1y = -intg[7];
            double r1z = -intg[9];
            double r2x = -intg[7];
            double r2z = -intg[8];
            double r3x = -intg[9];
            double r3y = -intg[8];
            
            //matrice tensore d'inerzia sul centro di massa
            if (bodyCoords)
            {
                r1x -= mass * (massCenter.y * massCenter.y + massCenter.z * massCenter.z);
                r2y -= mass * (massCenter.z * massCenter.z + massCenter.x * massCenter.x);
                r3z -= mass * (massCenter.x * massCenter.x + massCenter.y * massCenter.y);
                r1y += mass * massCenter.x * massCenter.y;
                r2z += mass * massCenter.y * massCenter.z;
                r1z += mass * massCenter.z * massCenter.x;
                r2x = r1y;
                r3y = r2z;
                r3x = r1z;
            }
		
			inertiaTensor = new Matrix3x3 (
				r1x, r1y, r1z,
				r2x, r2y, r2z,
				r3x, r3y, r3z);
			
			//L'oggetto ha massa totale 1, l'adatto alla massa richiesta
			double bufferMass = objMass / mass;

			inertiaTensor = inertiaTensor * bufferMass;
			
		}

		#endregion
	}
}

