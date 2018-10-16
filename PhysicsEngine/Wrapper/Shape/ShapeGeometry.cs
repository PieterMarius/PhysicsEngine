using SharpEngineMathUtility;
using SharpPhysicsEngine.ConvexHullWrapper;
using SharpPhysicsEngine.Helper;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Wrapper
{
    public sealed class ShapeGeometry
    {
        #region Fields

        private readonly CommonGeometry Geometry;

        #endregion

        #region Constructor

        public ShapeGeometry(
            Vector3d[] inputVertexPosition,
            int[][] inputTriangle)
        {
            TriangleMesh[] triangleMeshes = CommonUtilities.GetTriangleMeshes(inputTriangle);
            Geometry = new CommonGeometry(inputVertexPosition, triangleMeshes);
        }

        public ShapeGeometry(Vector3d[] inputVertexPosition)
        {
            IConvexHullEngine convexHullEngine = new ConvexHullEngine();

            ConvexHullData convexHullData = convexHullEngine.GetConvexHull(inputVertexPosition);
            TriangleMesh[] triangleMeshes = convexHullData.TriangleMeshes;
            Geometry = new CommonGeometry(
                Array.ConvertAll(convexHullData.Vertices, x => x.Vector3), 
                triangleMeshes);
        }

        #endregion

        #region Public Methods

        internal CommonGeometry GetGeometry()
        {
            return Geometry;
        }

        #endregion
    }
}
