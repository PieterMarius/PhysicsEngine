using SharpEngineMathUtility;
using SharpPhysicsEngine.Terrain;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class TerrainMesh
    {
        #region Fields

        HeightMapMesh terrain;

        #endregion

        #region Constructor

        public TerrainMesh()
        {
            terrain = new HeightMapMesh(0, 10, "heightmap.png",1);
        }

        #endregion

        #region Public Methods

        public double[][][] GetPositions()
        {
            return GeneralMathUtilities.GetMatrixFromVector3Matrix(terrain.GetPosition());
        }

        public double[][][] GetNormalArray()
        {
            return GeneralMathUtilities.GetMatrixFromVector3Matrix(terrain.GetNormalArray());
        }

        public double[][][] GetTextureCoordMatrix()
        {
            return GeneralMathUtilities.GetMatrixFromVector2Matrix(terrain.GetTextureCoords());
        }

        #endregion


    }
}
