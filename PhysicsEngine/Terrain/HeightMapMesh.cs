using SharpEngineMathUtility;
using System;
using System.Collections.Generic;
using System.IO;
using System.Windows.Media.Imaging;
using System.Drawing;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace SharpPhysicsEngine.Terrain
{
    internal class TerrainElement
    {
        #region Fields

        public List<Vector3> Position { get; private set; }
        public AABB Box { get; private set; }

        #endregion

        #region Constructor

        public TerrainElement()
        {
            Position = new List<Vector3>();
        }

        #endregion

        #region Public Methods

        public void AddPosition(Vector3 position)
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

        private int gridXDim = 20;
        private int gridZDim = 20;

        TerrainElement[,] terrainElements;

        private readonly Vector3[][] position;
        private readonly Vector3[][] normalsArr;
        private readonly Vector2[][] textureCoords;
        //private readonly int[] indicesArr;

        #endregion

        #region Constructor

        public HeightMapMesh(
            double minY, 
            double maxY, 
            String heightMapFile,
            int textInc,
            double scale)
        {
            this.minY = minY;
            this.maxY = maxY;
            this.scale = scale;

            Stream imageStreamSource = new FileStream(heightMapFile, FileMode.Open, FileAccess.Read, FileShare.Read);
            PngBitmapDecoder decoder = new PngBitmapDecoder(imageStreamSource, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);
            BitmapSource bitmapSource = decoder.Frames[0];
                       
            int height = bitmapSource.PixelHeight;
            int width = bitmapSource.PixelWidth;
           
            Byte[] bufferStream = new Byte[4 * bitmapSource.PixelHeight * bitmapSource.PixelWidth];
            
            bitmapSource.CopyPixels(bufferStream, width * 4, 0);

            double incx = (GetXLength() * scale) / (width - 1);
            double incz = (GetZLength() * scale) / (height - 1);
            double startx = -GetXLength() * 0.5 * scale;
            double startz = -GetZLength() * 0.5 * scale;
            double constStepX = GetXLength() * 0.5 * scale;
            double constStepZ = GetZLength() * 0.5 * scale;

            terrainElements = new TerrainElement[gridXDim, gridZDim];
            double positionIndexStepX = (GetXLength() * scale) / gridXDim;
            double positionIndexStepZ = (GetZLength() * scale) / gridZDim;

            Vector2[][] textCoords = new Vector2[height][];
            Vector3[][] positions = new Vector3[height][];
            List<Vector3> vPositions = new List<Vector3>();
            List<TriangleMesh> triangleMeshes = new List<TriangleMesh>();
                        
            for (int row = 0; row < height; row++)
            {
                positions[row] = new Vector3[width];
                textCoords[row] = new Vector2[width];

                for (int col = 0; col < width; col++)
                {
                    // Create vertex for current position
                    double pX = startx + col * incx;
                    double pZ = startz + row * incz;
                    double pY = GetHeight(col, row, width, bufferStream);
                    var vBuf = new Vector3(pX, pY, pZ);
                    
                    vPositions.Add(vBuf);
                    positions[row][col] = new Vector3(vBuf);
                                        
                    int idxX = Convert.ToInt32(Math.Floor((pX + constStepX) / positionIndexStepX));
                    int idxZ = Convert.ToInt32(Math.Floor((pZ + constStepZ) / positionIndexStepZ));

                    if (idxX == gridXDim)
                        idxX--;

                    if (idxZ == gridZDim)
                        idxZ--;

                    if (terrainElements[idxX, idxZ] == null)
                    {
                        terrainElements[idxX, idxZ] = new TerrainElement();    
                    }
                    terrainElements[idxX, idxZ].AddPosition(positions[row][col]);

                    // Set texture coordinates
                    textCoords[row][col] = new Vector2(
                        (double)textInc * (double)col / (double)width,
                        (double)textInc * (double)row / (double)height);

                    // Create indices
                    if (col < width - 1 && row < height - 1)
                    {
                        int leftTop = row * width + col;
                        int leftBottom = (row + 1) * width + col;
                        int rightBottom = (row + 1) * width + col + 1;
                        int rightTop = row * width + col + 1;

                        triangleMeshes.Add(new TriangleMesh(leftTop, leftBottom, rightTop));
                        triangleMeshes.Add(new TriangleMesh(rightTop, leftBottom, rightBottom));
                    }
                }
            }

            position = positions;
            //indicesArr = indices.ToArray();
            normalsArr = CalcNormals(position, width, height);
            textureCoords = textCoords;
        }

        public Vector3[][] GetPosition()
        {
            return position;
        }

        public Vector3[][] GetNormalArray()
        {
            return normalsArr;
        }

        public Vector2[][] GetTextureCoords()
        {
            return textureCoords;
        }

        #endregion

        #region Private Methods

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

        private Vector3[][] CalcNormals(Vector3[][] posArr, int width, int height)
        {
            Vector3 v0 = new Vector3();
            Vector3 v1 = new Vector3();
            Vector3 v2 = new Vector3();
            Vector3 v3 = new Vector3();
            Vector3 v4 = new Vector3();
            Vector3 v12 = new Vector3();
            Vector3 v23 = new Vector3();
            Vector3 v34 = new Vector3();
            Vector3 v41 = new Vector3();
            Vector3[][] normals = new Vector3[height][];
            Vector3 normal = new Vector3();
            for (int row = 0; row < height; row++)
            {
                normals[row] = new Vector3[width];
                for (int col = 0; col < width; col++)
                {
                    if (row > 0 && row < height - 1 && col > 0 && col < width - 1)
                    {
                        //int i0 = row * width * 3 + col * 3;
                        v0 = new Vector3(posArr[row][col]);

                        //int i1 = row * width * 3 + (col - 1) * 3;
                        v1 = new Vector3(posArr[row][col - 1]);
                        v1 = v1 - v0;
                        //v1 = v1.sub(v0);

                        //int i2 = (row + 1) * width * 3 + col * 3;
                        v2 = new Vector3(posArr[row + 1][col]);
                        v2 = v2 - v0;
                        //v2 = v2.sub(v0);

                        //int i3 = (row) * width * 3 + (col + 1) * 3;
                        v3 = new Vector3(posArr[row][col + 1]);
                        v3 = v3 - v0;
                        //v3 = v3.sub(v0);

                        //int i4 = (row - 1) * width * 3 + col * 3;
                        v4 = new Vector3(posArr[row - 1][col]);
                        v4 = v4 - v0;
                        //v4 = v4.sub(v0);

                        v12 = v1.Cross(v2).Normalize();
                        //v1.Cross(v2, v12);
                        //v12.normalize();

                        v23 = v2.Cross(v3).Normalize();
                        //v2.cross(v3, v23);
                        //v23.normalize();

                        v34 = v3.Cross(v4).Normalize();
                        //v3.cross(v4, v34);
                        //v34.normalize();

                        v41 = v4.Cross(v1).Normalize();
                        //v4.cross(v1, v41);
                        //v41.normalize();

                        normal = v12 + v23 + v34 + v41;

                        normals[row][col] = normal.Normalize();
                    }
                    else
                    {
                        normals[row][col] = new Vector3(0.0, 1.0, 0.0);
                    }
                    
                }
            }
            return normals;
        }

        #endregion
    }
}
