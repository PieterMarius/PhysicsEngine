using SharpEngineMathUtility;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;
using System.Drawing;
using System.Drawing.Imaging;

namespace SharpPhysicsEngine.Terrain
{
    internal sealed class HeightMapMesh
    {
        #region Fields

        private static readonly int MAX_COLOUR = 255 * 255 * 255;
        private static readonly double STARTX = -0.5;
        private static readonly double STARTZ = -0.5;

        private readonly double minY;
        private readonly double maxY;
        
        #endregion

        #region Constructor

        public HeightMapMesh(
            double minY, 
            double maxY, 
            String heightMapFile, 
            String textureFile, 
            int textInc)
        {
            this.minY = minY;
            this.maxY = maxY;

            Stream imageStreamSource = new FileStream(heightMapFile, FileMode.Open, FileAccess.Read, FileShare.Read);
            PngBitmapDecoder decoder = new PngBitmapDecoder(imageStreamSource, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);
            BitmapSource bitmapSource = decoder.Frames[0];
            Image img = Image.FromFile("");
            
            //PNGDecoder decoder = new PNGDecoder(getClass().getResourceAsStream(heightMapFile));
            int height = bitmapSource.PixelHeight;
            int width = bitmapSource.PixelWidth;
            MemoryStream buf = new MemoryStream(4 * bitmapSource.PixelHeight * bitmapSource.PixelWidth);
                        
            img.Save(buf, ImageFormat.Png);
            //ByteBuffer buf = ByteBuffer.allocateDirect(
            //        4 * decoder.getWidth() * decoder.getHeight());
            //decoder.
            //decoder.decode(buf, decoder.getWidth() * 4, PNGDecoder.Format.RGBA);
            //buf.flip();

            double incx = GetXLength() / (width - 1);
            double incz = GetZLength() / (height - 1);

            List<double> positions = new List<double>();
            List<double> textCoords = new List<double>();
            List<int> indices = new List<int>();

            for (int row = 0; row < height; row++)
            {
                for (int col = 0; col < width; col++)
                {
                    // Create vertex for current position
                    positions.Add(STARTX + col * incx); // x
                    positions.Add(GetHeight(col, row, width, buf)); //y
                    positions.Add(STARTZ + row * incz); //z

                    // Set texture coordinates
                    textCoords.Add((float)textInc * (float)col / (float)width);
                    textCoords.Add((float)textInc * (float)row / (float)height);

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
        }

        #endregion

        #region Private Methods

        private static double GetXLength()
        {
            return Math.Abs(-STARTX * 2);
        }

        private static double GetZLength()
        {
            return Math.Abs(-STARTX * 2);
        }

        private double GetHeight(int x, int z, int width, MemoryStream buffer)
        {
            var bufferStream = buffer.ToArray();
            byte r = bufferStream[x * 4 + 0 + z * 4 * width];
            byte g = bufferStream[x * 4 + 1 + z * 4 * width];
            byte b = bufferStream[x * 4 + 2 + z * 4 * width];
            byte a = bufferStream[x * 4 + 3 + z * 4 * width];
            int argb = ((0xFF & a) << 24) | ((0xFF & r) << 16)
                    | ((0xFF & g) << 8) | (0xFF & b);
            return minY + Math.Abs(maxY - minY) * ((double)argb / (double)MAX_COLOUR);
        }

        private double[] CalcNormals(double[] posArr, int width, int height)
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
            List<double> normals = new List<double>();
            Vector3 normal = new Vector3();
            for (int row = 0; row < height; row++)
            {
                for (int col = 0; col < width; col++)
                {
                    if (row > 0 && row < height - 1 && col > 0 && col < width - 1)
                    {
                        int i0 = row * width * 3 + col * 3;
                        v0 = new Vector3(posArr[i0], posArr[i0 + 1], posArr[i0 + 2]);
                        //v0.x = posArr[i0];
                        //v0.y = posArr[i0 + 1];
                        //v0.z = posArr[i0 + 2];

                        int i1 = row * width * 3 + (col - 1) * 3;
                        v1 = new Vector3(posArr[i1], posArr[i1 + 1], posArr[i1 + 2]);
                        v1 = v1 - v0;
                        //v1.x = posArr[i1];
                        //v1.y = posArr[i1 + 1];
                        //v1.z = posArr[i1 + 2];
                        //v1 = v1.sub(v0);

                        int i2 = (row + 1) * width * 3 + col * 3;
                        v2 = new Vector3(posArr[i2], posArr[i2 + 1], posArr[i2 + 2]);
                        v2 = v2 - v0;
                        //v2.x = posArr[i2];
                        //v2.y = posArr[i2 + 1];
                        //v2.z = posArr[i2 + 2];
                        //v2 = v2.sub(v0);

                        int i3 = (row) * width * 3 + (col + 1) * 3;
                        v3 = new Vector3(posArr[i3], posArr[i3 + 1], posArr[i3 + 2]);
                        v3 = v3 - v0;
                        //v3.x = posArr[i3];
                        //v3.y = posArr[i3 + 1];
                        //v3.z = posArr[i3 + 2];
                        //v3 = v3.sub(v0);

                        int i4 = (row - 1) * width * 3 + col * 3;
                        v4 = new Vector3(posArr[i4], posArr[i4 + 1], posArr[i4 + 2]);
                        v4 = v4 - v0;
                        //v4.x = posArr[i4];
                        //v4.y = posArr[i4 + 1];
                        //v4.z = posArr[i4 + 2];
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
                        
                        normal = normal.Normalize();
                    }
                    else
                    {
                        normal = new Vector3(0.0, 1.0, 0.0);
                    }
                    normal = normal.Normalize();
                    normals.Add(normal.x);
                    normals.Add(normal.y);
                    normals.Add(normal.z);
                }
            }
            return normals.ToArray();
        }

        #endregion
    }
}
