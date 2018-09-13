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
using System;
using System.Collections.Generic;
using System.IO;
using System.Windows.Media.Imaging;
using System.Drawing;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using System.Threading.Tasks;
using System.Linq;
using SharpPhysicsEngine.ConvexHullWrapper;

namespace SharpPhysicsEngine.Terrain
{
    internal class TerrainElement
    {
        #region Fields

        public List<Vector3d> Position { get; private set; }
        public AABB Box { get; private set; }

        #endregion

        #region Constructor

        public TerrainElement()
        {
            Position = new List<Vector3d>();
        }

        #endregion

        #region Public Methods

        public void AddPosition(Vector3d position)
        {
            Position.Add(position);
        }

        #endregion

        #region Private Methods

        private void GenerateShapes()
        {
            //ShapeConvexDecomposition convexDecomposition = new ShapeConvexDecomposition(Box, TriangleMeshes);

            //Vertex3Index[] verticesIndex = Array.ConvertAll(InputVertexPosition, x => new Vertex3Index(x, null, -1));
            //var convexShapes = convexDecomposition.GetConvexShapeList(verticesIndex, 0.2);

            //ConvexShapesGeometry = new Geometry[convexShapes.Count];


            //ConvexHullData convexHullData = convexHullEngine.GetConvexHull(convexVertices);

            //ConvexShapesGeometry[i] = new Geometry(
            //    this,
            //    convexHullData.Vertices,
            //    convexHullData.TriangleMeshes,
            //    ObjectGeometryType.ConvexShape,
            //            true);
        }

        #endregion

    }

    internal sealed class HeightMapMesh
    {
        #region Fields

        private static readonly int MAX_COLOUR = 255 * 255 * 255;
        private static readonly double STARTX = -0.5;
        private static readonly double STARTZ = -0.5;

        private readonly double minY;
        private readonly double maxY;
        private readonly double scale;
        private int Height;
        private int Width;

        private int gridXDim = 20;
        private int gridZDim = 20;

        private TerrainElement[,] terrainElements;

        private readonly Vector3d[] positions;
        private readonly Vector3d[] normalsArr;
        private readonly Vector2d[][] textureCoords;
        private double incx;
        private double incz;
        private double startx;
        private double startz;

        private TriangleMesh[] triangleMeshes;
        private List<ShapeDecompositionOutput> convexShapes;

        //private readonly int[] indicesArr;

        #endregion

        #region Constructor

        public HeightMapMesh(
            String heightMapFile,
            IConvexHullEngine convexHullEngine,
            double minY,
            double maxY,
            int textInc,
            double scale)
        {
            this.minY = minY;
            this.maxY = maxY;
            this.scale = scale;

            var bufferStream = LoadBitmap(heightMapFile);
                        
            textureCoords = new Vector2d[Height][];
            positions = new Vector3d[Height * Width];
            Init(bufferStream, textInc);
            //indicesArr = indices.ToArray();
            normalsArr = CalcNormals(positions);
            GenerateConvexShapes(convexHullEngine);
        }

        #endregion

        #region Public Methods

        public Vector3d[] GetPosition()
        {
            return positions;
        }

        public Vector3d[] GetNormalArray()
        {
            return normalsArr;
        }

        public Vector2d[][] GetTextureCoords()
        {
            return textureCoords;
        }

        public Vector3d[][] GetConvexShapes()
        {
            Vector3d[][] result = new Vector3d[convexShapes.Count][];

            for (int i = 0; i < convexShapes.Count; i++)
                result[i] = convexShapes[i].Vertex3Idx.Select(x => x.Vector3).ToArray();

            return result;
        }

        public void GetPotentialCollidingShape(AABB box)
        {
            //Cercare il posIndex (estrarre 4 punti più bassi) 
            var min = box.Min;
            var max = box.Max;

            //Point 1
            var col = (int)Math.Floor((min.x - startx) / incx);
            var row = (int)Math.Floor((min.z - startz) / incz);
            int posIndex = row * Width + col;
            var heigth = positions[posIndex].z;

            //Point 2
            var col1 = (int)Math.Floor((min.x + max.x - startx) / incx);
            var row1 = (int)Math.Floor((min.z - startz) / incz);
            int posIndex1 = row * Width + col;
            var heigth1 = positions[posIndex1].z;

            //Point 3
            var col2 = (int)Math.Floor((min.x - startx) / incx);
            var row2 = (int)Math.Floor((min.z + max.z - startz) / incz);
            int posIndex2 = row * Width + col;
            var heigth2 = positions[posIndex2].z;
        
            //Point 4
            var col3 = (int)Math.Floor((min.x + max.x - startx) / incx);
            var row3 = (int)Math.Floor((min.z + max.z - startz) / incz);          
            int posIndex3 = row * Width + col;
            var heigth3 = positions[posIndex3].z;
        }

        #endregion

        #region Private Methods

        private Byte[] LoadBitmap(string heightMapFile)
        {
            Stream imageStreamSource = new FileStream(heightMapFile, FileMode.Open, FileAccess.Read, FileShare.Read);
            PngBitmapDecoder decoder = new PngBitmapDecoder(imageStreamSource, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);
            BitmapSource bitmapSource = decoder.Frames[0];

            Height = bitmapSource.PixelHeight;
            Width = bitmapSource.PixelWidth;

            Byte[] bufferStream = new Byte[4 * bitmapSource.PixelHeight * bitmapSource.PixelWidth];

            bitmapSource.CopyPixels(bufferStream, Width * 4, 0);

            return bufferStream;
        }

        private void Init(byte[] bufferStream, int textInc)
        {
            incx = (GetXLength() * scale) / (Width - 1);
            incz = (GetZLength() * scale) / (Height - 1);
            startx = -GetXLength() * 0.5 * scale;
            startz = -GetZLength() * 0.5 * scale;
            double constStepX = GetXLength() * 0.5 * scale;
            double constStepZ = GetZLength() * 0.5 * scale;

            terrainElements = new TerrainElement[gridXDim, gridZDim];
            double positionIndexStepX = (GetXLength() * scale) / gridXDim;
            double positionIndexStepZ = (GetZLength() * scale) / gridZDim;

            List<TriangleMesh> triangle = new List<TriangleMesh>();

            for (int row = 0; row < Height; row++)
            {
                textureCoords[row] = new Vector2d[Width];

                for (int col = 0; col < Width; col++)
                {
                    // Create vertex for current position
                    double pX = startx + col * incx;
                    double pZ = startz + row * incz;
                    double pY = GetHeight(col, row, Width, bufferStream);
                    var vBuf = new Vector3d(pX, pY, pZ);

                    int posIndex = row * Width + col;
                    positions[posIndex] = new Vector3d(vBuf);

                    // Set texture coordinates
                    textureCoords[row][col] = new Vector2d(
                        (double)textInc * (double)col / (double)Width,
                        (double)textInc * (double)row / (double)Height);

                    // Create indices
                    if (col < Width - 1 && row < Height - 1)
                    {
                        int leftTop = posIndex;
                        int leftBottom = (row + 1) * Width + col;
                        int rightBottom = (row + 1) * Width + col + 1;
                        int rightTop = row * Width + col + 1;

                        triangle.Add(new TriangleMesh(leftTop, leftBottom, rightTop));
                        triangle.Add(new TriangleMesh(rightTop, leftBottom, rightBottom));
                    }

                    // Terrain Subdivision
                    int idxX = Convert.ToInt32(Math.Floor((pX + constStepX) / positionIndexStepX));
                    int idxZ = Convert.ToInt32(Math.Floor((pZ + constStepZ) / positionIndexStepZ));

                    idxX = (idxX == gridXDim) ? idxX - 1 : idxX;
                    idxZ = (idxZ == gridZDim) ? idxZ - 1 : idxZ;
                    
                    if (terrainElements[idxX, idxZ] == null)
                    {
                        terrainElements[idxX, idxZ] = new TerrainElement();
                    }
                    terrainElements[idxX, idxZ].AddPosition(vBuf);
                }
            }

            triangleMeshes = triangle.ToArray();
        }

        private byte[] ImageToByteArray(Image imageIn)
        {
            using (var ms = new MemoryStream())
            {
                imageIn.Save(ms, imageIn.RawFormat);
                return ms.ToArray();
            }
        }

        private static byte[] ImageToByte(Image img)
        {
            ImageConverter converter = new ImageConverter();
            return (byte[])converter.ConvertTo(img, typeof(byte[]));
        }

        private static double GetXLength()
        {
            return Math.Abs(-STARTX * 2);
        }

        private static double GetZLength()
        {
            return Math.Abs(-STARTZ * 2);
        }

        private double GetHeight(int x, int z, int width, byte[] bufferStream)
        {
            int index = (z * width + x) * 4;
            var r = bufferStream[index];
            var g = bufferStream[index + 1];
            var b = bufferStream[index + 2];
            var a = bufferStream[index + 3];
            long rgb = MakeRgb(r, g, b);
            return minY + Math.Abs(maxY - minY) * ((double)rgb / (double)MAX_COLOUR);
        }

        private static long MakeArgb(byte alpha, byte red, byte green, byte blue)
        {
            return (long)(((ulong)((((red << 0x10) | (green << 8))
                | blue) | (alpha << 0x18))) & 0xffffffffL);
        }

        private static long MakeRgb(byte red, byte green, byte blue)
        {
            return (long)(((ulong)((((red << 0x10) | (green << 8))
                | blue))) & 0xffffffffL);
        }

        private Vector3d[] CalcNormals(Vector3d[] posArr)
        {
            Vector3d v0 = new Vector3d();
            Vector3d v1 = new Vector3d();
            Vector3d v2 = new Vector3d();
            Vector3d v3 = new Vector3d();
            Vector3d v4 = new Vector3d();
            Vector3d v12 = new Vector3d();
            Vector3d v23 = new Vector3d();
            Vector3d v34 = new Vector3d();
            Vector3d v41 = new Vector3d();
            Vector3d[] normals = new Vector3d[Height * Width];
            Vector3d normal = new Vector3d();

            for (int row = 0; row < Height; row++)
            {
                for (int col = 0; col < Width; col++)
                {
                    if (row > 0 && row < Height - 1 && col > 0 && col < Width - 1)
                    {
                        int posIndex = row * Width + col;
                        v0 = new Vector3d(posArr[posIndex]);

                        posIndex = row * Width + col - 1;
                        v1 = new Vector3d(posArr[posIndex]);
                        v1 = v1 - v0;

                        posIndex = (row + 1) * Width + col;
                        v2 = new Vector3d(posArr[posIndex]);
                        v2 = v2 - v0;
                        
                        posIndex = row * Width + col + 1;
                        v3 = new Vector3d(posArr[posIndex]);
                        v3 = v3 - v0;
                        
                        posIndex = (row - 1) * Width + col;
                        v4 = new Vector3d(posArr[posIndex]);
                        v4 = v4 - v0;
                        
                        v12 = v1.Cross(v2).Normalize();
                        
                        v23 = v2.Cross(v3).Normalize();
                        
                        v34 = v3.Cross(v4).Normalize();
                        
                        v41 = v4.Cross(v1).Normalize();
                        
                        normal = v12 + v23 + v34 + v41;

                        normals[row * Height + col] = normal.Normalize();
                    }
                    else
                    {
                        normals[row * Height + col] = new Vector3d(0.0, 1.0, 0.0);
                    }

                }
            }
            return normals;
        }

        private void GenerateConvexShapes(IConvexHullEngine convexHullEngine)
        {
            AABB box = AABB.GetGeometryAABB(positions, this);
            
            var vertex3Idx = SetVertexAdjacency(positions, triangleMeshes);
            ConvexDecompositionEngine convexDecomposition = new ConvexDecompositionEngine(box, vertex3Idx, 0.7);

            convexShapes = convexDecomposition.Execute().GetConvexShapeList(true);
        }

        private Vertex3Index[] SetVertexAdjacency(
            Vector3d[] inputVertexPosition,
            TriangleMesh[] triangle)
        {
            Vertex3Index[] vertexPosition = Array.ConvertAll(inputVertexPosition, x => new Vertex3Index(x, new HashSet<int>(), -1));

            foreach (var tr in triangle)
            {
                vertexPosition[tr.a].AddIndex(tr.b);
                vertexPosition[tr.a].AddIndex(tr.c);
                vertexPosition[tr.b].AddIndex(tr.a);
                vertexPosition[tr.b].AddIndex(tr.c);
                vertexPosition[tr.c].AddIndex(tr.a);
                vertexPosition[tr.c].AddIndex(tr.b);
            }

            return vertexPosition;
        }

        #endregion
    }
}
