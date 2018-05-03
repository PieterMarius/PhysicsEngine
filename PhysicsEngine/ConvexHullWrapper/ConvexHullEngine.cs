using System;
using System.Linq;
using MIConvexHull;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.ConvexHullWrapper
{
    internal sealed class ConvexHullEngine : IConvexHullEngine
    {        
        #region Public Methods

        public ConvexHullData GetConvexHull(Vector3[] points)
        {
            ConvexHullVertex[] vtx = new ConvexHullVertex[points.Length];

            for (int i = 0; i < points.Length; i++)
                vtx[i] = new ConvexHullVertex() { Position = points[i].Array, Index = i };
            
            try
            {
                ConvexHull<ConvexHullVertex, DefaultConvexFace<ConvexHullVertex>> cHull = ConvexHull.Create(vtx);
                var faces = cHull.Faces.ToArray();

                TriangleMesh[] triangleMeshes = new TriangleMesh[faces.Length];
                Vector3[] vertices = Array.ConvertAll(cHull.Points.ToArray(), x=> new Vector3(x.Position));

                foreach (var face in faces.Select((value, i) => new { value, i }))
                {
                    triangleMeshes[face.i] = new TriangleMesh(
                        face.value.Vertices[0].Index,
                        face.value.Vertices[1].Index,
                        face.value.Vertices[2].Index);
                }
                    
                return new ConvexHullData(vertices,triangleMeshes);
            }
            catch (Exception ex)
            {
                throw new Exception(ex.Message, ex);
            }
        }

        #endregion
    }
}
