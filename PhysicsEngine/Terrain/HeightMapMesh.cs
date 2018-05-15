using SharpEngineMathUtility;
using System;
using System.Collections.Generic;
using System.IO;
using System.Windows.Media.Imaging;
using System.Drawing;

namespace SharpPhysicsEngine.Terrain
{
    internal sealed class HeightMapMesh
    {
        #region Fields

        private static readonly int MAX_COLOUR = 255 * 255 * 255;
        private static readonly double STARTX = -10.0;
        private static readonly double STARTZ = -10.0;

        private readonly double minY;
        private readonly double maxY;

        private readonly Vector3[][] position;
        private readonly Vector3[][] normalsArr;
        private readonly Vector2[][] textureCoords;
        private readonly int[] indicesArr;

        #endregion

        #region Constructor

        public HeightMapMesh(
            double minY, 
            double maxY, 
            String heightMapFile,
            int textInc)
        {
            this.minY = minY;
            this.maxY = maxY;

            Stream imageStreamSource = new FileStream(heightMapFile, FileMode.Open, FileAccess.Read, FileShare.Read);
            PngBitmapDecoder decoder = new PngBitmapDecoder(imageStreamSource, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);
            BitmapSource bitmapSource = decoder.Frames[0];
                       
            int height = bitmapSource.PixelHeight;
            int width = bitmapSource.PixelWidth;
           
            Byte[] bufferStream = new Byte[4 * bitmapSource.PixelHeight * bitmapSource.PixelWidth];
            
            bitmapSource.CopyPixels(bufferStream, width * 4, 0);

            double incx = GetXLength() / (width - 1);
            double incz = GetZLength() / (height - 1);

            Vector2[][] textCoords = new Vector2[height][];
            Vector3[][] positions = new Vector3[height][];

            List<int> indices = new List<int>();

            for (int row = 0; row < height; row++)
            {
                positions[row] = new Vector3[width];
                textCoords[row] = new Vector2[width];

                for (int col = 0; col < width; col++)
                {
                    // Create vertex for current position
                    positions[row][col] = new Vector3(
                        STARTX + col * incx,
                        GetHeight(col, row, width, bufferStream),
                        STARTZ + row * incz);
                                        
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

                        indices.Add(leftTop);
                        indices.Add(leftBottom);
                        indices.Add(rightTop);

                        indices.Add(rightTop);
                        indices.Add(leftBottom);
                        indices.Add(rightBottom);
                    }
                }
            }

            position = positions;
            indicesArr = indices.ToArray();
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
