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
using SharpPhysicsEngine.NonConvexDecomposition.Hierarchical_Tree;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    internal sealed class ConvexDecompositionEngine : IShapeConvexDecomposition
    {
        #region Fields

        #region Hierarchical Properties

        private AABB region;

        private readonly ConvexDecompositionEngine[] childNode = new ConvexDecompositionEngine[8];

        private List<Vertex3Index> VertexPosition;

        private ConvexDecompositionEngine parent;

        #endregion

        private const double perturbationValue =  1E-5;

        private readonly double DecompositionValue;

        private Vertex3Index[] BaseVertexPosition;
        
        #endregion

        #region Constructor

        private ConvexDecompositionEngine(
            AABB region,
            List<Vertex3Index> vertexPosition,
            double precisionSize,
            bool internalConstructor)
        {
            DecompositionValue = precisionSize;
            this.region = region;
            VertexPosition = vertexPosition;
        }

        public ConvexDecompositionEngine(
            AABB region,
            Vertex3Index[] vertexPosition,
            double precisionSize)
        {
            DecompositionValue = precisionSize;
            BaseVertexPosition = vertexPosition;
            this.region = region;
            VertexPosition = vertexPosition.ToList();
        }

        #endregion

        #region Public Methods

        public ConcaveHierarchicalTree Execute()
        {
            return Decompose();
        }
                
        //public List<ShapeDecompositionOutput> GetIntersectedShape(
        //    AABB box,
        //    AABB region,
        //    Vertex3Index[] vertexPosition,
        //    double precisionSize,
        //    double distanceTolerance)
        //{
        //    VertexPosition = vertexPosition.ToList();
        //    BaseVertexPosition = vertexPosition;

        //    this.region = region;

        //    DecompositionValue = precisionSize;
            
        //    BuildTree();

        //    List<ShapeDecompositionOutput> convexShapes = new List<ShapeDecompositionOutput>();
                        
        //    FindIntersectedConvexShape(box, ref convexShapes, distanceTolerance);

        //    if (convexShapes.Count > 0)
        //    {
        //        FinalizeShape(ref convexShapes);
        //        return convexShapes;
        //    }

        //    return null;
        //}
        
        internal static AABB FindEnclosingCube(AABB region)
        {
            //we can't guarantee that all bounding regions will be relative to the origin, so to keep the math
            //simple, we're going to translate the existing region to be oriented off the origin and remember the translation.
            //find the min offset from (0,0,0) and translate by it for a short while
            Vector3d offset = Vector3d.Zero() - region.Min;
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
                    region.Max = new Vector3d(highX, highX, highX);

                    region.Min -= offset;
                    region.Max -= offset;
                    return new AABB(region.Min, region.Max, null);
                }
            }

            //We have a cube with non-power of two dimensions. We want to find the next highest power of two.
            //example: 63 -> 64; 65 -> 128;
            int x = GeneralMathUtils.SigBit(highX);

            region.Max = new Vector3d(x, x, x);

            region.Min -= offset;
            region.Max -= offset;

            return new AABB(region.Min, region.Max, null);
        }

        #endregion

        #region Private Methods

        private ConcaveHierarchicalTree Decompose()
        {
            BuildTree();

            List<ShapeDecompositionOutput> convexShapes = new List<ShapeDecompositionOutput>();

            ConcaveHierarchicalTree hTree = new ConcaveHierarchicalTree
            {
                ChildNodes = new List<HierarchicalTree<Vertex3Index, AABB>>()
            };

            BuildHierachicalTree(ref hTree, this);

            return hTree;
        }

        private void BuildTree()
        {
            //terminate the recursion if we're a leaf node
            if (VertexPosition.Count <= 1)
                return;

            Vector3d dimensions = region.Max - region.Min;

            if (dimensions == Vector3d.Zero())
            {
                region = FindEnclosingCube(region);
                dimensions = region.Max - region.Min;
            }

            //Check to see if the dimensions of the box are greater than the minimum dimensions
            if (dimensions.x <= DecompositionValue && dimensions.y <= DecompositionValue && dimensions.z <= DecompositionValue)
            {
                return;
            }

            Vector3d half = dimensions * 0.5;
            Vector3d center = region.Min + half;

            //Create subdivided regions for each octant
            AABB[] octant = new AABB[8];
            octant[0] = new AABB(region.Min, center, null);
            octant[1] = new AABB(new Vector3d(center.x, region.Min.y, region.Min.z), new Vector3d(region.Max.x, center.y, center.z), null);
            octant[2] = new AABB(new Vector3d(center.x, region.Min.y, center.z), new Vector3d(region.Max.x, center.y, region.Max.z), null);
            octant[3] = new AABB(new Vector3d(region.Min.x, region.Min.y, center.z), new Vector3d(center.x, center.y, region.Max.z), null);
            octant[4] = new AABB(new Vector3d(region.Min.x, center.y, region.Min.z), new Vector3d(center.x, region.Max.y, center.z), null);
            octant[5] = new AABB(new Vector3d(center.x, center.y, region.Min.z), new Vector3d(region.Max.x, region.Max.y, center.z), null);
            octant[6] = new AABB(center, region.Max, null);
            octant[7] = new AABB(new Vector3d(region.Min.x, center.y, center.z), new Vector3d(center.x, region.Max.y, region.Max.z), null);

            //This will contain all of our objects which fit within each respective octant.
            List<Vertex3Index>[] octList = new List<Vertex3Index>[8];
            List<Vertex3Index>[] delist = new List<Vertex3Index>[8];

            for (int i = 0; i < 8; i++)
            {
                octList[i] = new List<Vertex3Index>();
                delist[i] = new List<Vertex3Index>();
            }
            
            //this list contains all of the objects which got moved down the tree and can be delisted from this node.
            foreach (Vertex3Index point in VertexPosition)
            {
                for (int i = 0; i < 8; i++)
                {
                    if (octant[i].Contains(point.Vector3))
                    {
                        octList[i].Add(point);
                        delist[i].Add(point);
                        break;
                    }
                }
            }
            
            //Create child nodes where there are items contained in the bounding region
            for (int i = 0; i < 8; i++)
            {
                if (octList[i].Count > 0)
                {
                    foreach (Vertex3Index obj in delist[i])
                        VertexPosition.Remove(obj);

                    //delist every moved object from this node.
                    childNode[i] = CreateNode(octant[i], octList[i]);
                    childNode[i].BuildTree();
                }
            }
        }
                
        private ConvexDecompositionEngine CreateNode(AABB region, List<Vertex3Index> objList)
        {
            if (objList.Count == 0)
                return null;

            ConvexDecompositionEngine ret = new ConvexDecompositionEngine(region, objList, DecompositionValue, true);
            {
                parent = this;
            };

            return ret;
        }

        private void BuildHierachicalTree(
            ref ConcaveHierarchicalTree tree, 
            ConvexDecompositionEngine decompositionTree)
        {
            tree.ChildNodes = new List<HierarchicalTree<Vertex3Index, AABB>>();
            tree.Elements = new List<Vertex3Index>(decompositionTree.VertexPosition);
            tree.TotalElements = new List<Vertex3Index>(BaseVertexPosition);
            
            for (int i = 0; i < 8; i++)
            {
                if (decompositionTree.childNode[i] != null)
                {
                    tree.ChildNodes.Add(new ConcaveHierarchicalTree { Parent = tree });
                    var bufTree = (ConcaveHierarchicalTree)tree.ChildNodes[tree.ChildNodes.Count - 1];
                    BuildHierachicalTree(ref bufTree, decompositionTree.childNode[i]);
                }
            }
        }

        #endregion
    }
}
