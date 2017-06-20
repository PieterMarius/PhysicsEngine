using ConvexHullGenerator;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public sealed class ShapeConvexDecomposition
    {
        #region Fields

        private double MIN_SIZE = 0.5;

        private AABB region;

        private ShapeConvexDecomposition[] childNode = new ShapeConvexDecomposition[8];

        private List<Vertex3Index> VertexPosition;

        private ShapeConvexDecomposition parent;

        private byte m_activeNodes = 0;

        private readonly TriangleIndexes[] triangleIndexes;

        private readonly Vertex3Index[] BaseVertexPosition;

        #endregion

        #region Constructor

        private ShapeConvexDecomposition(
            AABB region,
            List<Vertex3Index> vertexPosition,
            TriangleIndexes[] triangleIndexes,
            double precisionSize)
        {
            MIN_SIZE = precisionSize;
            this.region = region;
            this.triangleIndexes = triangleIndexes;
            VertexPosition = vertexPosition;
        }

        public ShapeConvexDecomposition(
            AABB region,
            Vertex3Index[] vertexPosition,
            TriangleIndexes[] triangleIndexes)
        {
            this.region = region;
            this.triangleIndexes = triangleIndexes;
            VertexPosition = vertexPosition.ToList();
            BaseVertexPosition = new List<Vertex3Index>(vertexPosition).ToArray();
        }

        #endregion

        #region Public Methods

        public static AABB FindEnclosingCube(AABB region)
        {
            //we can't guarantee that all bounding regions will be relative to the origin, so to keep the math
            //simple, we're going to translate the existing region to be oriented off the origin and remember the translation.
            //find the min offset from (0,0,0) and translate by it for a short while
            Vector3 offset = Vector3.Zero() - region.Min;
            region.Min += offset;
            region.Max += offset;

            //A 3D rectangle has a length, height, and width. Of those three dimensions, we want to find the largest dimension.
            //the largest dimension will be the minimum dimensions of the cube we're creating.
            int highX = (int)System.Math.Ceiling(System.Math.Max(System.Math.Max(region.Max.x, region.Max.y), region.Max.z));

            //see if our cube dimension is already at a power of 2. If it is, we don't have to do any work.
            for (int bit = 0; bit < 32; bit++)
            {
                if (highX == 1 << bit)
                {
                    region.Max = new Vector3(highX, highX, highX);

                    region.Min -= offset;
                    region.Max -= offset;
                    return new AABB(region.Min, region.Max);
                }
            }

            //We have a cube with non-power of two dimensions. We want to find the next highest power of two.
            //example: 63 -> 64; 65 -> 128;
            int x = GeneralMathUtilities.SigBit(highX);

            region.Max = new Vector3(x, x, x);

            region.Min -= offset;
            region.Max -= offset;

            return new AABB(region.Min, region.Max);
        }
        
        public Vertex3Index[][][] GetConvexShapeList(double precisionSize)
        {
            MIN_SIZE = precisionSize;

            BuildTree();

            List<List<Vertex3Index>> convexShapes = new List<List<Vertex3Index>>();

            GenerateConvexShapeList(ref convexShapes);

            return FinalizeShape(convexShapes);
        }
        
        #endregion

        #region Private Methods

        private void BuildTree()
        {
            //terminate the recursion if we're a leaf node
            if (VertexPosition.Count <= 1)
                return;

            Vector3 dimensions = region.Max - region.Min;

            if (dimensions == Vector3.Zero())
            {
                region = FindEnclosingCube(region);
                dimensions = region.Max - region.Min;
            }

            //Check to see if the dimensions of the box are greater than the minimum dimensions
            if (dimensions.x <= MIN_SIZE && dimensions.y <= MIN_SIZE && dimensions.z <= MIN_SIZE)
            {
                return;
            }

            Vector3 half = dimensions / 2.0f;
            Vector3 center = region.Min + half;

            //Create subdivided regions for each octant
            AABB[] octant = new AABB[8];
            octant[0] = new AABB(region.Min, center);
            octant[1] = new AABB(new Vector3(center.x, region.Min.y, region.Min.z), new Vector3(region.Max.x, center.y, center.z));
            octant[2] = new AABB(new Vector3(center.x, region.Min.y, center.z), new Vector3(region.Max.x, center.y, region.Max.z));
            octant[3] = new AABB(new Vector3(region.Min.x, region.Min.y, center.z), new Vector3(center.x, center.y, region.Max.z));
            octant[4] = new AABB(new Vector3(region.Min.x, center.y, region.Min.z), new Vector3(center.x, region.Max.y, center.z));
            octant[5] = new AABB(new Vector3(center.x, center.y, region.Min.z), new Vector3(region.Max.x, region.Max.y, center.z));
            octant[6] = new AABB(center, region.Max);
            octant[7] = new AABB(new Vector3(region.Min.x, center.y, center.z), new Vector3(center.x, region.Max.y, region.Max.z));

            //This will contain all of our objects which fit within each respective octant.
            List<Vertex3Index>[] octList = new List<Vertex3Index>[8];
            List<Vertex3Index> delist = new List<Vertex3Index>();

            for (int i = 0; i < 8; i++)
                octList[i] = new List<Vertex3Index>();
            
            //this list contains all of the objects which got moved down the tree and can be delisted from this node.
            foreach (Vertex3Index point in VertexPosition)
            {
                for (int a = 0; a < 8; a++)
                {
                    if (octant[a].Contains(point.Vector3))
                    {
                        octList[a].Add(point);
                        delist.Add(point);
                        break;
                    }
                }
            }

            foreach (Vertex3Index obj in delist)
                VertexPosition.Remove(obj);

            //Create child nodes where there are items contained in the bounding region
            for (int a = 0; a < 8; a++)
            {
                if (octList[a].Count > 0)
                {
                    //delist every moved object from this node.
                    childNode[a] = CreateNode(octant[a], octList[a]);
                    m_activeNodes |= (byte)(1 << a);
                    childNode[a].BuildTree();
                }
            }

        }
                
        private ShapeConvexDecomposition CreateNode(AABB region, List<Vertex3Index> objList)
        {
            if (objList.Count == 0)
                return null;

            ShapeConvexDecomposition ret = new ShapeConvexDecomposition(region, objList, triangleIndexes, MIN_SIZE);
            ret.parent = this;

            return ret;
        }

        private void GenerateConvexShapeList(ref List<List<Vertex3Index>> geometry)
        {
            if (VertexPosition.Count > 0)
                geometry.Add(new List<Vertex3Index>(VertexPosition));

            for (int a = 0; a < 8; a++)
            {
                if (childNode[a] != null)
                    childNode[a].GenerateConvexShapeList(ref geometry);
            }
        }

        private Vertex3Index[][][] FinalizeShape(List<List<Vertex3Index>> convexShapes)
        {
            var convexHullShape = new Vertex3Index[convexShapes.Count][][];

            foreach (var shape in convexShapes.Select((value, i) => new { value, i }))
            {
                List<Vertex3Index> bufVertex = new List<Vertex3Index>();
                HashSet<int> triIdx = new HashSet<int>();

                foreach (var item in shape.value)
                {
                    for (int i = 0; i < item.Indexes.Length; i++)
                    {
                        if (triIdx.Add(triangleIndexes[item.Indexes[i]].a))
                            bufVertex.Add(BaseVertexPosition[triangleIndexes[item.Indexes[i]].a]);
                        if (triIdx.Add(triangleIndexes[item.Indexes[i]].b))
                            bufVertex.Add(BaseVertexPosition[triangleIndexes[item.Indexes[i]].b]);
                        if (triIdx.Add(triangleIndexes[item.Indexes[i]].c))
                            bufVertex.Add(BaseVertexPosition[triangleIndexes[item.Indexes[i]].c]);
                    }
                }
                shape.value.AddRange(bufVertex);

                List<IVertex> vtx = new List<IVertex>(shape.value);

                ConvexHull<IVertex, DefaultConvexFace<IVertex>>  cHull = ConvexHull.Create(vtx.ToList());
                                
                convexHullShape[shape.i] = Array.ConvertAll(cHull.Faces.ToArray(), x =>
                    new Vertex3Index[] {
                        (Vertex3Index)x.Vertices[0],
                        (Vertex3Index)x.Vertices[1],
                        (Vertex3Index)x.Vertices[2] });
            }

            return convexHullShape;
        }

        #endregion
    }
}
