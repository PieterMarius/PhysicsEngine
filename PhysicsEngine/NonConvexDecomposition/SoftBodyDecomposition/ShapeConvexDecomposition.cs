/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * PhysicsEngine, Copyright (c) 2018 Pieter Marius van Duin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 *****************************************************************************/

using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    internal sealed class ShapeConvexDecomposition : IShapeConvexDecomposition
    {
        #region Fields

        private const double perturbationValue =  1E-5;

        private double DecompositionValue;

        private AABB region;

        private ShapeConvexDecomposition[] childNode = new ShapeConvexDecomposition[8];

        private List<Vertex3Index> VertexPosition;

        private ShapeConvexDecomposition parent;

        private Vertex3Index[] BaseVertexPosition;

        private readonly TriangleIndexes[] triangleIndexes;
        
        #endregion

        #region Constructor

        private ShapeConvexDecomposition(
            AABB region,
            List<Vertex3Index> vertexPosition,
            TriangleIndexes[] triangleIndexes,
            double precisionSize)
        {
            DecompositionValue = precisionSize;
            this.region = region;
            this.triangleIndexes = triangleIndexes;
            VertexPosition = vertexPosition;
        }

        public ShapeConvexDecomposition(
            AABB region,
            TriangleIndexes[] triangleIndexes)
        {
            this.region = region;
            this.triangleIndexes = triangleIndexes;            
        }

        public ShapeConvexDecomposition(
            TriangleIndexes[] triangleIndexes)
        {
            this.triangleIndexes = triangleIndexes;
        }

        #endregion

        #region Public Methods

        internal static AABB FindEnclosingCube(AABB region)
        {
            //we can't guarantee that all bounding regions will be relative to the origin, so to keep the math
            //simple, we're going to translate the existing region to be oriented off the origin and remember the translation.
            //find the min offset from (0,0,0) and translate by it for a short while
            Vector3 offset = Vector3.Zero() - region.Min;
            region.Min += offset;
            region.Max += offset;

            //A 3D rectangle has a length, height, and width. Of those three dimensions, we want to find the largest dimension.
            //the largest dimension will be the minimum dimensions of the cube we're creating.
            int highX = (int)Math.Ceiling(Math.Max(Math.Max(region.Max.x, region.Max.y), region.Max.z));

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
        
        public List<ShapeDecompositionOutput> GetConvexShapeList(
            Vertex3Index[] vertexPosition,
            double precisionSize)
        {
            return Decompose(vertexPosition, region, precisionSize);
        }

        public List<ShapeDecompositionOutput> GetConvexShapeList(
            Vertex3Index[] vertexPosition,
            AABB region,
            double precisionSize)
        {
            return Decompose(vertexPosition, region, precisionSize);
        }

        public List<ShapeDecompositionOutput> GetIntersectedShape(
            AABB box,
            AABB region,
            Vertex3Index[] vertexPosition,
            double precisionSize,
            double distanceTolerance)
        {
            VertexPosition = vertexPosition.ToList();
            BaseVertexPosition = vertexPosition;

            this.region = region;

            DecompositionValue = precisionSize;
            
            BuildTree();

            List<ShapeDecompositionOutput> convexShapes = new List<ShapeDecompositionOutput>();

            FindIntersectedConvexShape(box, ref convexShapes, distanceTolerance);

            if (convexShapes.Count > 0)
            {
                FinalizeShape(ref convexShapes);
                return convexShapes;
            }

            return null;
        }
        
        #endregion

        #region Private Methods

        private List<ShapeDecompositionOutput> Decompose(
            Vertex3Index[] vertexPosition,
            AABB region,
            double precisionSize)
        {
            VertexPosition = vertexPosition.ToList();
            BaseVertexPosition = vertexPosition;

            DecompositionValue = precisionSize;

            BuildTree();

            List<ShapeDecompositionOutput> convexShapes = new List<ShapeDecompositionOutput>();

            GenerateConvexShapeList(ref convexShapes);

            if (convexShapes.Count > 0)
            {
                FinalizeShape(ref convexShapes);
                return convexShapes;
            }

            throw new Exception("Convex Decomposition Failed.");
        }

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
            if (dimensions.x <= DecompositionValue && dimensions.y <= DecompositionValue && dimensions.z <= DecompositionValue)
            {
                return;
            }

            Vector3 half = dimensions * 0.5;
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
                    childNode[a].BuildTree();
                }
            }
        }
                
        private ShapeConvexDecomposition CreateNode(AABB region, List<Vertex3Index> objList)
        {
            if (objList.Count == 0)
                return null;

            ShapeConvexDecomposition ret = new ShapeConvexDecomposition(region, objList, triangleIndexes, DecompositionValue)
            {
                parent = this
            };

            return ret;
        }

        private void GenerateConvexShapeList(ref List<ShapeDecompositionOutput> geometry)
        {
            if (VertexPosition.Count > 0)
                geometry.Add(new ShapeDecompositionOutput(new HashSet<Vertex3Index>(VertexPosition), region));

            for (int a = 0; a < 8; a++)
            {
                if (childNode[a] != null)
                    childNode[a].GenerateConvexShapeList(ref geometry);
            }
        }

        private void FindIntersectedConvexShape(AABB box, ref List<ShapeDecompositionOutput> geometry, double distanceTolerance)
        {
            if (region.Intersect(box, distanceTolerance))
            {
                if (VertexPosition.Count > 0)
                    geometry.Add(new ShapeDecompositionOutput(new HashSet<Vertex3Index>(VertexPosition), region));

                for (int a = 0; a < 8; a++)
                {
                    if (childNode[a] != null)
                        childNode[a].FindIntersectedConvexShape(box, ref geometry, distanceTolerance);
                }
            }
        }

        private void FinalizeShape(ref List<ShapeDecompositionOutput> convexShapes)
        {
             Parallel.ForEach(convexShapes,
                            new ParallelOptions { MaxDegreeOfParallelism = 2 },
                            shape =>
             {
                 HashSet<Vertex3Index> bufVertex = new HashSet<Vertex3Index>();
                 
                 foreach (var vertex in shape.Vertex3Idx)
                 {
                     foreach (var idx in vertex.Indexes)
                     {
                         bufVertex.Add(BaseVertexPosition[triangleIndexes[idx].a]);
                         bufVertex.Add(BaseVertexPosition[triangleIndexes[idx].b]);
                         bufVertex.Add(BaseVertexPosition[triangleIndexes[idx].c]);
                     }
                 }

                 shape.AddVertex3Index(bufVertex);
                                  
                 if (shape.Vertex3Idx.Count <= 3)
                 {
                     bufVertex.Clear();
                     for (int i = 0; i < 4 - shape.Vertex3Idx.Count; i++)
                     {
                         bufVertex.Add(new Vertex3Index(
                             shape.Vertex3Idx.First().Vector3 + Vector3.Random(-perturbationValue, perturbationValue), 
                             new int[] { int.MaxValue - i }, 
                             0));
                     }
                     shape.AddVertex3Index(bufVertex);
                 }
             });
        }

        #endregion
    }
}
