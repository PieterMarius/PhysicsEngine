using SharpEngineMathUtility;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class CommonGeometry
    {
        #region Fields

        public Vector3d[] VerticesPosition { get; private set; }
        public SupportIndex[] VerticesIdx { get; private set; }
        public TriangleMesh[] Triangle { get; private set; }

        #endregion

        #region Constructor

        public CommonGeometry(
            Vector3d[] vertexPosition,
            TriangleMesh[] triangle)
        {
            VerticesPosition = vertexPosition;
            Triangle = triangle;
            SetVertexAdjacency(Enumerable.Range(0, vertexPosition.Length).ToArray());
        }

        internal CommonGeometry(
            Vector3d[] vertexPosition,
            TriangleMesh[] triangle,
            int[] verticesIdx)
        {
            VerticesPosition = vertexPosition;
            Triangle = triangle;
            SetVertexAdjacency(verticesIdx);
        }

        #endregion

        #region Public Methods

        #endregion

        #region Private Methods

        private void SetVertexAdjacency(int[] verticesIdx)
        {
            VerticesIdx = Array.ConvertAll(verticesIdx, x => new SupportIndex(x));

            if (Triangle != null)
            {
                var vList = VerticesIdx.ToList();

                foreach (var tr in Triangle)
                {
                    var indexA = vList.FindIndex(x => x.ID == tr.a);
                    var indexB = vList.FindIndex(x => x.ID == tr.b);
                    var indexC = vList.FindIndex(x => x.ID == tr.c);

                    vList[indexA].AddVertexToGlobalAdjList(tr.b);
                    vList[indexA].AddVertexToGlobalAdjList(tr.c);
                    vList[indexB].AddVertexToGlobalAdjList(tr.a);
                    vList[indexB].AddVertexToGlobalAdjList(tr.c);
                    vList[indexC].AddVertexToGlobalAdjList(tr.a);
                    vList[indexC].AddVertexToGlobalAdjList(tr.b);

                    vList[indexA].AddVertexToLocalAdjList(indexB);
                    vList[indexA].AddVertexToLocalAdjList(indexC);
                    vList[indexB].AddVertexToLocalAdjList(indexA);
                    vList[indexB].AddVertexToLocalAdjList(indexC);
                    vList[indexC].AddVertexToLocalAdjList(indexA);
                    vList[indexC].AddVertexToLocalAdjList(indexB);
                }

                VerticesIdx = vList.ToArray();
            }
        }

        #endregion

    }
}
