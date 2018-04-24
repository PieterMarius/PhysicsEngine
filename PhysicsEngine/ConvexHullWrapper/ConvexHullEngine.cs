using System;
using System.Collections.Generic;
using System.Linq;
using ConvexHullGenerator;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.ConvexHullWrapper
{
    internal sealed class ConvexHullEngine
    {
        public ConvexHullEngine()
        {

        }

        #region Public Methods

        public VertexProperties[] GetConvexHull(Vector3[] points)
        {
            VertexProperties[] result = new VertexProperties[points.Length];

            IVertex[] vtx = Array.ConvertAll(points, x => new DefaultVertex() { Position = x.Array });

            try
            {
                ConvexHull<IVertex, DefaultConvexFace<IVertex>> cHull = ConvexHull.Create(vtx);
                var faces = cHull.Faces.ToArray();
                
                foreach (var face in faces.Select((value, i) => new { value, i }))
                {
                    int index = face.i * 3;
                    
                    result[index] = new VertexProperties(new Vector3(face.value.Vertices[0].Position));
                    result[index + 1] = new VertexProperties(new Vector3(face.value.Vertices[1].Position));
                    result[index + 2] = new VertexProperties(new Vector3(face.value.Vertices[2].Position));
                }

                //cHull.Faces.ToArray()[0].Adjacency[0].

                var convexHullShape = Array.ConvertAll(cHull.Faces.ToArray(), x => x.Vertices);

                var convert = Array.ConvertAll(convexHullShape, x => Array.ConvertAll(x, y => new Vector3(y.Position)));

                return null;
            }
            catch (Exception ex)
            {
                throw new Exception(ex.Message);
            }
        }

        #endregion
    }
}
