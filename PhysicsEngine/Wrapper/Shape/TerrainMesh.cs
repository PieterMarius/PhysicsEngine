using SharpEngineMathUtility;
using SharpPhysicsEngine.ConvexHullWrapper;
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
            IConvexHullEngine convexHullEngine = new ConvexHullEngine();
            terrain = new HeightMapMesh(
                "heightmap.png", 
                convexHullEngine, 
                0, 
                10, 
                1, 
                100);
        }

        #endregion

        #region Public Methods

        public double[][] GetPositions()
        {
            return GeneralMathUtilities.GetArrayFromVector3(terrain.GetPosition());
        }

        public double[][] GetNormalArray()
        {
            return GeneralMathUtilities.GetArrayFromVector3(terrain.GetNormalArray());
        }

        public double[][][] GetTextureCoordMatrix()
        {
            return GeneralMathUtilities.GetMatrixFromVector2Matrix(terrain.GetTextureCoords());
        }

        public double[][][] GetConvexShapeList()
        {
            return GeneralMathUtilities.GetMatrixFromVector3Matrix(terrain.GetConvexShapes());
        }

        #endregion


    }
}
