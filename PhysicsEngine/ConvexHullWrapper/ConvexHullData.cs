using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.ConvexHullWrapper
{
    internal sealed class ConvexHullData
    {
        #region Fields

        public Vector3[] Vertices { get; private set; }

        public TriangleMesh[] TriangleMeshes { get; private set; }

        #endregion

        #region Constructor

        public ConvexHullData(
            Vector3[] vertices,
            TriangleMesh[] triangleMeshes)
        {
            Vertices = vertices;
            TriangleMeshes = triangleMeshes;
        }

        #endregion
    }
}
