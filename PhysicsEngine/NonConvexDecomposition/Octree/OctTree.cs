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
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.NonConvexDecomposition.Octree
{
    internal sealed class OctTree
    {
        #region Fields

        /// <summary>
        /// The minumum size for enclosing region is a 1x1x1 cube.
        /// </summary>
        private const double MIN_SIZE = 0.005;

        private AABB region;

        private OctTree[] childNode = new OctTree[8];

        private readonly Vector3[] VertexPosition;

        private readonly List<TriangleMesh> baseTriangles;

        private List<TriangleMesh> triangles;

        private OctTree parent;

        private byte m_activeNodes = 0;

        private static bool treeBuilt = false;       //there is no pre-existing tree yet.

        #endregion

        #region Constructor

        public OctTree(
            AABB region, 
            List<TriangleMesh> triangles, 
            Vector3[] vertexPosition)
        {
            this.region = region;
            baseTriangles = triangles;
            this.triangles = new List<TriangleMesh>(triangles);
            VertexPosition = vertexPosition;
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

        public void GetConvexShapeList(IShape shape, ref List<IGeometry> geometry)
        {
            HashSet<Vector3> convexPoint = new HashSet<Vector3>();
            List<TriangleMesh> inputTriangle = new List<TriangleMesh>();

            foreach (var triangle in triangles)
            {
                convexPoint.Add(VertexPosition[triangle.a]);
                convexPoint.Add(VertexPosition[triangle.b]);
                convexPoint.Add(VertexPosition[triangle.c]);

                inputTriangle.Add(new TriangleMesh(triangle.a, triangle.b, triangle.c));
            }

            if (inputTriangle.Count > 0)
            {
                geometry.Add(new Geometry(shape, convexPoint.ToArray(), inputTriangle.ToArray(), ObjectGeometryType.ConvexBody, false));
            }

            for (int a = 0; a < 8; a++)
            {
                if (childNode[a] != null)
                    childNode[a].GetConvexShapeList(shape, ref geometry);
            }
            
        }

        public void Render(ref List<Line> boxLines)
        {
            Vector3[] verts = new Vector3[8];
            verts[0] = region.Min;
            verts[1] = new Vector3(region.Min.x, region.Min.y, region.Max.z); //Z
            verts[2] = new Vector3(region.Min.x, region.Max.y, region.Min.z); //Y
            verts[3] = new Vector3(region.Max.x, region.Min.y, region.Min.z); //X

            verts[7] = region.Max;
            verts[4] = new Vector3(region.Max.x, region.Max.y, region.Min.z); //Z
            verts[5] = new Vector3(region.Max.x, region.Min.y, region.Max.z); //Y
            verts[6] = new Vector3(region.Min.x, region.Max.y, region.Max.z); //X

            boxLines.Add(new Line(verts[0], verts[1]));
            boxLines.Add(new Line(verts[0], verts[2]));
            boxLines.Add(new Line(verts[0], verts[3]));
            boxLines.Add(new Line(verts[7], verts[4]));
            boxLines.Add(new Line(verts[7], verts[5]));
            boxLines.Add(new Line(verts[7], verts[6]));
            
            boxLines.Add(new Line(verts[1], verts[6]));
            boxLines.Add(new Line(verts[1], verts[5]));
            boxLines.Add(new Line(verts[4], verts[2]));
            boxLines.Add(new Line(verts[4], verts[3]));
            boxLines.Add(new Line(verts[2], verts[6]));
            boxLines.Add(new Line(verts[3], verts[5]));

            for (int a = 0; a < 8; a++)
            {
                if (childNode[a] != null)
                    childNode[a].Render(ref boxLines);
            }
        }

        public void BuildOctree()
        {
            if (!treeBuilt)
                BuildTree();
            else
            {
                triangles = new List<TriangleMesh>(triangles);
                BuildTree();
            }
        }

        #endregion

        #region Private Methods

        private void BuildTree()
        {
            //terminate the recursion if we're a leaf node
            if (triangles.Count <= 4)
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
            List<TriangleMesh>[] octList = new List<TriangleMesh>[8];
            HashSet<TriangleMesh> delist = new HashSet<TriangleMesh>();

            for (int i = 0; i < 8; i++)
            {
                octList[i] = new List<TriangleMesh>();
                
            }

            //this list contains all of the objects which got moved down the tree and can be delisted from this node.
            //TODO da modificare per gestire il numero max di oggetti in ogni bounding box

            foreach (TriangleMesh triangle in triangles)
            {
                AABB boundingBox = AABB.GetTriangleAABB(new Vector3[3] {
                    VertexPosition[triangle.a],
                    VertexPosition[triangle.b],
                    VertexPosition[triangle.c] });

                if (boundingBox.Min != boundingBox.Max)
                {
                    for (int a = 0; a < 8; a++)
                    {
                        if (octant[a].Contains(boundingBox) )
                        {
                            octList[a].Add(triangle);
                            delist.Add(triangle);
                            break;
                        }
                    }
                }
            }
                        

            //delist every moved object from this node.
            foreach (TriangleMesh obj in delist)
                triangles.Remove(obj);

            //foreach (TriangleIndexes triangle in triangles)
            //{
            //    AABB boundingBox = AABB.GetTriangleAABB(new Vector3[3] {
            //        VertexPosition[triangle.a],
            //        VertexPosition[triangle.b],
            //        VertexPosition[triangle.c] });

            //    if (boundingBox.Min != boundingBox.Max)
            //    {
            //        for (int a = 0; a < 8; a++)
            //        {
            //            if (octant[a].Intersect(boundingBox))
            //            {
            //                octList[a].Add(triangle);
            //                delist.Add(triangle);
            //                break;
            //            }
            //        }
            //    }
            //}

            //foreach (TriangleIndexes obj in delist)
            //    triangles.Remove(obj);

            //Create child nodes where there are items contained in the bounding region
            for (int a = 0; a < 8; a++)
            {
                if (octList[a].Count > 0)
                {
                    childNode[a] = CreateNode(octant[a], octList[a]);
                    m_activeNodes |= (byte)(1 << a);
                    childNode[a].BuildTree();
                }
            }

            treeBuilt = true;
        }

        private OctTree CreateNode(AABB region, List<TriangleMesh> objList)  //complete & tested
        {
            if (objList.Count == 0)
                return null;

            OctTree ret = new OctTree(region, objList, VertexPosition)
            {
                parent = this
            };

            return ret;
        }

        private OctTree CreateNode(AABB region, TriangleMesh Item)
        {
            List<TriangleMesh> objList = new List<TriangleMesh>(1)
            {
                Item
            }; //sacrifice potential CPU time for a smaller memory footprint

            OctTree ret = new OctTree(region, objList, VertexPosition)
            {
                parent = this
            };
            return ret;
        }

        //private void UpdateTree()   //complete & tested
        //{
        //    /*I think I can just directly insert items into the tree instead of using a queue.*/
        //    if (!treeBuilt)
        //    {
        //        while (pendingInsertion.Count != 0)
        //            triangles.Add(pendingInsertion.Dequeue());

        //        //trim out any objects which have the exact same bounding areas

        //        BuildTree();
        //    }
        //    else
        //    {
        //        while (pendingInsertion.Count != 0)
        //            Insert(pendingInsertion.Dequeue());
        //    }

        //    treeReady = true;
        //}

        #endregion
    }
}
