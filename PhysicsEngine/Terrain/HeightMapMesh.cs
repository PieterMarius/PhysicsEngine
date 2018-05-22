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
        private int Height;
        private int Width;

        private int gridXDim = 20;
        private int gridZDim = 20;

        private TerrainElement[,] terrainElements;

        private readonly Vector3[] positions;
        private readonly Vector3[] normalsArr;
        private readonly Vector2[][] textureCoords;
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
                        
            textureCoords = new Vector2[Height][];
            positions = new Vector3[Height * Width];
            Init(bufferStream, textInc);
            //indicesArr = indices.ToArray();
            normalsArr = CalcNormals(positions);
            GenerateConvexShapes(convexHullEngine);
        }

        public Vector3[] GetPosition()
        {
            return positions;
        }

        public Vector3[] GetNormalArray()
        {
            return normalsArr;
        }

        public Vector2[][] GetTextureCoords()
        {
            return textureCoords;
        }

        public Vector3[][] GetConvexShapes()
        {
            Vector3[][] result = new Vector3[convexShapes.Count][];

            for (int i = 0; i < convexShapes.Count; i++)
                result[i] = convexShapes[i].Vertex3Idx.Select(x => x.Vector3).ToArray();

            return result;
        }

        #endregion

        #region Private Methods

        private Byte[] LoadBitmap(string heightMapFile)
        {
            Stream imageStreamSource = new FileStream(heightMapFile, FileMode.Open, FileAccess.Read, FileShare.Read);
            PngBitmapDecoder decoder = new PngBitmapDecoder(imageStreamSource, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);
            BitmapSource bitmapSource = decoder.Frames[0];

            this.Height = bitmapSource.PixelHeight;
            this.Width = bitmapSource.PixelWidth;

            Byte[] bufferStream = new Byte[4 * bitmapSource.PixelHeight * bitmapSource.PixelWidth];

            bitmapSource.CopyPixels(bufferStream, Width * 4, 0);

            return bufferStream;
        }

        private void Init(byte[] bufferStream, int textInc)
        {
            double incx = (GetXLength() * scale) / (Width - 1);
            double incz = (GetZLength() * scale) / (Height - 1);
            double startx = -GetXLength() * 0.5 * scale;
            double startz = -GetZLength() * 0.5 * scale;
            double constStepX = GetXLength() * 0.5 * scale;
            double constStepZ = GetZLength() * 0.5 * scale;

            terrainElements = new TerrainElement[gridXDim, gridZDim];
            double positionIndexStepX = (GetXLength() * scale) / gridXDim;
            double positionIndexStepZ = (GetZLength() * scale) / gridZDim;

            List<Vector3> vPositions = new List<Vector3>();
            List<TriangleMesh> triangle = new List<TriangleMesh>();

            for (int row = 0; row < Height; row++)
            {
                textureCoords[row] = new Vector2[Width];

                for (int col = 0; col < Width; col++)
                {
                    // Create vertex for current position
                    double pX = startx + col * incx;
                    double pZ = startz + row * incz;
                    double pY = GetHeight(col, row, Width, bufferStream);
                    var vBuf = new Vector3(pX, pY, pZ);

                    vPositions.Add(vBuf);
                    int posIndex = row * Width + col;
                    positions[posIndex] = new Vector3(vBuf);

                    // Set texture coordinates
                    textureCoords[row][col] = new Vector2(
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

                    if (idxX == gridXDim)
                        idxX--;

                    if (idxZ == gridZDim)
                        idxZ--;

                    if (terrainElements[idxX, idxZ] == null)
                    {
                        terrainElements[idxX, idxZ] = new TerrainElement();
                    }
                    terrainElements[idxX, idxZ].AddPosition(vBuf);
                }
            }

            triangleMeshes = triangle.ToArray();
            //indicesArr = indices.ToArray();
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

        private Vector3[] CalcNormals(Vector3[] posArr)
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
            Vector3[] normals = new Vector3[Height * Width];
            Vector3 normal = new Vector3();

            for (int row = 0; row < Height; row++)
            {
                for (int col = 0; col < Width; col++)
                {
                    if (row > 0 && row < Height - 1 && col > 0 && col < Width - 1)
                    {
                        int posIndex = row * Width + col;
                        v0 = new Vector3(posArr[posIndex]);

                        posIndex = row * Width + col - 1;
                        v1 = new Vector3(posArr[posIndex]);
                        v1 = v1 - v0;
                        //v1 = v1.sub(v0);

                        posIndex = (row + 1) * Width + col;
                        v2 = new Vector3(posArr[posIndex]);
                        v2 = v2 - v0;
                        //v2 = v2.sub(v0);

                        posIndex = row * Width + col + 1;
                        v3 = new Vector3(posArr[posIndex]);
                        v3 = v3 - v0;
                        //v3 = v3.sub(v0);

                        posIndex = (row - 1) * Width + col;
                        v4 = new Vector3(posArr[posIndex]);
                        v4 = v4 - v0;
                        //v4 = v4.sub(v0);

                        v12 = v1.Cross(v2).Normalize();
                        //v1.Cross(v2, v12);

                        v23 = v2.Cross(v3).Normalize();
                        //v2.cross(v3, v23);

                        v34 = v3.Cross(v4).Normalize();
                        //v3.cross(v4, v34);

                        v41 = v4.Cross(v1).Normalize();
                        //v4.cross(v1, v41);

                        normal = v12 + v23 + v34 + v41;

                        normals[row * Height + col] = normal.Normalize();
                    }
                    else
                    {
                        normals[row * Height + col] = new Vector3(0.0, 1.0, 0.0);
                    }

                }
            }
            return normals;
        }

        private void GenerateConvexShapes(IConvexHullEngine convexHullEngine)
        {
            AABB box = AABB.GetGeometryAABB(positions);
            ShapeConvexDecomposition convexDecomposition = new ShapeConvexDecomposition(box, triangleMeshes);
            var vertex3Idx = SetVertexAdjacency(positions, triangleMeshes);

            convexShapes = convexDecomposition.GetConvexShapeList(vertex3Idx, 0.2);
        }

        private Vertex3Index[] SetVertexAdjacency(
            Vector3[] inputVertexPosition,
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
