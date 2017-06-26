using System;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;
using OpenTK.Graphics.OpenGL;
using ObjLoader.Loader.Loaders;
using System.Collections.Generic;
using SharpEngineMathUtility;

namespace Utility
{
	public static class OpenGLUtilities
	{
		
		public static LoadResult LoadObjSolid(string fileName)
		{
			var objLoaderFactory = new ObjLoaderFactory ();
			var objLoader = objLoaderFactory.Create ();
			var fileStream = new FileStream (fileName, FileMode.OpenOrCreate);
			LoadResult solid = objLoader.Load (fileStream);
			fileStream.Close();
			return solid;
		}

		public static int LoadTexture(string filename)
		{
			var bitmap = new Bitmap (filename);

			int id = GL.GenTexture ();

			BitmapData bmpData = bitmap.LockBits (
				new Rectangle (0, 0, bitmap.Width, bitmap.Height),
				ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

			GL.BindTexture (TextureTarget.Texture2D, id);

			GL.TexImage2D (TextureTarget.Texture2D, 0,
				PixelInternalFormat.Rgba,
				bitmap.Width, bitmap.Height, 0,
				OpenTK.Graphics.OpenGL.PixelFormat.Bgra,
				PixelType.UnsignedByte,
				bmpData.Scan0);

			bitmap.UnlockBits (bmpData);

			GL.TexParameter (TextureTarget.Texture2D,
				TextureParameterName.TextureMinFilter, 
				(int)TextureMinFilter.Linear);

			GL.TexParameter (TextureTarget.Texture2D,
				TextureParameterName.TextureMagFilter, 
				(int)TextureMagFilter.Linear);

			return id;
		}

		/// <summary>
		/// Loads the GL objects into OpenGL buffer.
		/// </summary>
		/// <returns>The GL objects.</returns>
		/// <param name="objects">Objects.</param>
		/// <param name="nObject">N object.</param>
		public static int[][] LoadGLObjects(
			ObjImporter.meshStruct[][] objects,
			SharpEngineMathUtility.Vector3[][] translate,
			int nObject,
			bool GLM_TEXTURE = false, 
			bool GLM_FLAT = false,
			bool GLM_SMOOTH = false )
		{
			int[][] displayList = new int[nObject][]; 

			for (int i = 0; i < nObject; i++) 
			{
				displayList[i] = new int[objects[i].Length];

				for (int j = 0; j < objects[i].Length; j++)
				{
					displayList[i][j] = GL.GenLists(1);

					GL.NewList(displayList[i][j], ListMode.CompileAndExecute);

					GLDrawSolid(objects[i][j], translate[i][j], GLM_TEXTURE, GLM_FLAT, GLM_SMOOTH);

					GL.EndList();
				}
			}

			return displayList;
		}

		/// <summary>
		/// Loads the GL objects into OpenGL buffer.
		/// </summary>
		/// <returns>The GL objects.</returns>
		/// <param name="objects">Objects.</param>
		/// <param name="nObject">N object.</param>
		public static int[][] LoadGLObjects(
			ObjImporter.meshStruct[][] objects,
			int nObject,
			bool GLM_TEXTURE = false,
			bool GLM_FLAT = false,
			bool GLM_SMOOTH = false)
		{
			int[][] displayList = new int[nObject][];

			for (int i = 0; i < nObject; i++)
			{
				displayList[i] = new int[objects[i].Length];

				for (int j = 0; j < objects[i].Length; j++)
				{
					displayList[i][j] = GL.GenLists(1);

					GL.NewList(displayList[i][j], ListMode.CompileAndExecute);

					GLDrawSolid(objects[i][j], new SharpEngineMathUtility.Vector3(), GLM_TEXTURE, GLM_FLAT, GLM_SMOOTH);

					GL.EndList();
				}
			}

			return displayList;
		}

		public static float UnitizeObject(ref LoadResult obj)
		{
			float xMax, xMin, yMax, yMin, zMax, zMin;
			float cx, cy, cz, w, h, d;
			float scale;

			xMax = obj.Vertices [0].X;
			xMin = obj.Vertices [0].X;
			yMax = obj.Vertices [0].Y;
			yMin = obj.Vertices [0].Y;
			zMax = obj.Vertices [0].Z;
			zMin = obj.Vertices [0].Z;

			for (int i = 1; i < obj.Vertices.Count; i++) 
			{
				if (xMax < obj.Vertices [i].X)
					xMax = obj.Vertices [i].X;
				if (xMin > obj.Vertices [i].X)
					xMin = obj.Vertices [i].X;

				if (yMax < obj.Vertices [i].Y)
					yMax = obj.Vertices [i].Y;
				if (yMin > obj.Vertices [i].Y)
					yMin = obj.Vertices [i].Y;

				if (zMax < obj.Vertices [i].Z)
					zMax = obj.Vertices [i].Z;
				if (zMin > obj.Vertices [i].Z)
					zMin = obj.Vertices [i].Z;
			}

			/* calculate model width, height, and depth */
			w = Math.Abs (xMax) + Math.Abs (xMin);
			h = Math.Abs (yMax) + Math.Abs (yMin);
			d = Math.Abs (zMax) + Math.Abs (zMin);

			/* calculate center of the model */
			cx = (xMax + xMin) / 2.0f;
			cy = (yMax + yMin) / 2.0f;
			cz = (zMax + zMin) / 2.0f;

			scale = 2.0f / Math.Max (Math.Max (w, h), d);

			/* translate around center then scale */
			for (int i = 0; i < obj.Vertices.Count; i++) 
			{
				var vertex = new ObjLoader.Loader.Data.VertexData.Vertex (
					(obj.Vertices [i].X - cx) * scale,
					(obj.Vertices [i].Y - cy) * scale,
					(obj.Vertices [i].Z - cz) * scale);

				obj.Vertices [i] = vertex;
			}

			return scale;
		}

		public static double UnitizeObject(ref SharpEngineMathUtility.Vector3[] vertices)
		{
			double xMax, xMin, yMax, yMin, zMax, zMin;
			double cx, cy, cz, w, h, d;
			double scale;

			xMax = vertices[0].x;
			xMin = vertices[0].x;
			yMax = vertices[0].y;
			yMin = vertices[0].y;
			zMax = vertices[0].z;
			zMin = vertices[0].z;

			for (int i = 1; i < vertices.Length; i++)
			{
				if (xMax < vertices[i].x)
					xMax = vertices[i].x;
				if (xMin > vertices[i].x)
					xMin = vertices[i].x;

				if (yMax < vertices[i].y)
					yMax = vertices[i].y;
				if (yMin > vertices[i].y)
					yMin = vertices[i].y;

				if (zMax < vertices[i].z)
					zMax = vertices[i].z;
				if (zMin > vertices[i].z)
					zMin = vertices[i].z;
			}

			/* calculate model width, height, and depth */
			w = Math.Abs(xMax) + Math.Abs(xMin);
			h = Math.Abs(yMax) + Math.Abs(yMin);
			d = Math.Abs(zMax) + Math.Abs(zMin);

			/* calculate center of the model */
			cx = (xMax + xMin) / 2.0f;
			cy = (yMax + yMin) / 2.0f;
			cz = (zMax + zMin) / 2.0f;

			scale = 2.0f / Math.Max(Math.Max(w, h), d);

			/* translate around center then scale */
			for (int i = 0; i < vertices.Length; i++)
			{
				var vertex = new SharpEngineMathUtility.Vector3(
					(vertices[i].x - cx) * scale,
					(vertices[i].y - cy) * scale,
					(vertices[i].z - cz) * scale);

				vertices[i] = vertex;
			}

			return scale;
		}

		public static void ScaleObject(
			ref LoadResult obj, 
			float scale)
		{
			for (int i = 0; i < obj.Vertices.Count; i++) 
			{
				var vertex = new ObjLoader.Loader.Data.VertexData.Vertex (
					(obj.Vertices [i].X) * scale,
					(obj.Vertices [i].Y) * scale,
					(obj.Vertices [i].Z) * scale);

				obj.Vertices [i] = vertex;
			}
		}

		public static void ScaleObject(
			ref SharpEngineMathUtility.Vector3[] vertices,
			double scale)
		{
			for (int i = 0; i < vertices.Length; i++)
			{
				var vertex = new SharpEngineMathUtility.Vector3(
					(vertices[i].x) * scale,
					(vertices[i].y) * scale,
					(vertices[i].z) * scale);

				vertices[i] = vertex;
			}
		}

		#region GLU Utilities

		public static float[] GluOrtho2D(
			float left,
			float right,
			float bottom,
			float top)
		{
			float zNear = -1.0f;
			float zFar = 1.0f;
			float inv_z = 1.0f / (zFar - zNear);
			float inv_y = 1.0f / (top - bottom);
			float inv_x = 1.0f / (right - left);

			float[] m = new float[16];

			m [0] = 2.0f * inv_x; m[1]  = 0; 			m [2] = 0; 				m [3] = -(right + left) * inv_x;

			m [4] = 0; 			  m [5] = 2.0f * inv_y; m [6] = 0; 				m [7] = -(top + bottom) * inv_y;

			m [8] = 0; 			  m [9] = 0; 			m [10] = -2.0f * inv_z;  m [11] = -(zFar + zNear) * inv_z;

			m [12] = 0; 		  m [13] = 0; 			m [14] = 0; 			 m [15] = 1.0f;

			return m;

		}

		#endregion

		public static void GluPerspective( 
			double fovy, 
			double aspect, 
			double zNear, 
			double zFar)
		{
			double xmin, xmax, ymin, ymax;

			ymax = zNear * Math.Tan( fovy * Math.PI / 360.0 );
			ymin = -ymax;
			xmin = ymin * aspect;
			xmax = ymax * aspect;

			GL.LoadIdentity ();
			GL.Frustum( xmin, xmax, ymin, ymax, zNear, zFar );
		}


		public static float[] BuildPerspProjMat(
			float fov, 
			float aspect,
			float znear, 
			float zfar)
		{

			float[] m = new float[16];
			float xymax = Convert.ToSingle(znear * Math.Tan(fov * 0.5 * Math.PI/180));
			float xmin = -xymax;

			float width = xymax - xmin;
			float height = xymax + xymax;

			float depth = zfar - znear;
			float q = -(zfar + znear) / depth;
			float qn = -2 * (zfar * znear) / depth;

			float w = 2 * znear / width;
			w = w / aspect;
			float h = 2 * znear / height;

			m[0]  = w; m[1]  = 0; m[2]  = 0; m[3]  = 0;

			m[4]  = 0; m[5]  = h; m[6]  = 0; m[7]  = 0;

			m[8]  = 0; m[9]  = 0; m[10] = q; m[11] = -1;

			m[12] = 0; m[13] = 0; m[14] = qn; m[15] = 0;

			return m;
		}

		public static void drawPoint(float size)
		{
			GL.Begin(PrimitiveType.Points);
			GL.Enable(EnableCap.PointSprite);
			GL.PointSize(size);
			GL.Vertex3(0.0, 0.0, 0.0);
			GL.End();
		}

		public static void drawSolidCube(float size)
		{
			float[][] n = new float[6] [];
			n [0] = new float[] { -1.0f, 0.0f, 0.0f };
			n [1] = new float[] { 0.0f, 1.0f, 0.0f};
			n [2] = new float[] { 1.0f, 0.0f, 0.0f};
			n [3] = new float[] { 0.0f, -1.0f, 0.0f};
			n [4] = new float[] { 0.0f, 0.0f, 1.0f};
			n [5] = new float[] { 0.0f, 0.0f, -1.0f};

			int[][] faces = new int[6] [];
			faces [0] = new int[] { 0, 1, 2, 3 };
			faces [1] = new int[] { 3, 2, 6, 7 };
			faces [2] = new int[] { 7, 6, 5, 4 };
			faces [3] = new int[] { 4, 5, 1, 0 };
			faces [4] = new int[] { 5, 6, 2, 1 };
			faces [5] = new int[] { 7, 4, 0, 3 };

			float bSize = size / 2.0f;
			float[][] v = new float[8] [];
			v [0] = new float[] { -bSize, -bSize, -bSize };
			v [1] = new float[] { -bSize, -bSize, bSize };
			v [2] = new float[] { -bSize, bSize, bSize };
			v [3] = new float[] { -bSize, bSize, -bSize };
			v [4] = new float[] { bSize, -bSize, -bSize };
			v [5] = new float[] { bSize, -bSize, bSize };
			v [6] = new float[] { bSize, bSize, bSize };
			v [7] = new float[] { bSize, bSize, -bSize };

			GL.Begin (PrimitiveType.Quads);
			for (int i = 5; i >= 0; i--) 
			{
				GL.Normal3 (n [i] [0],n[i][1],n[i][2]);
				GL.Vertex3 (v [faces [i] [0]] [0], v [faces [i] [0]] [1], v [faces [i] [0]] [2]);
				GL.Vertex3 (v [faces [i] [1]] [0], v [faces [i] [1]] [1], v [faces [i] [1]] [2]);
				GL.Vertex3 (v [faces [i] [2]] [0], v [faces [i] [2]] [1], v [faces [i] [2]] [2]);
				GL.Vertex3 (v [faces [i] [3]] [0], v [faces [i] [3]] [1], v [faces [i] [3]] [2]);

			}
			GL.End ();
		}

		public static List<Line> BuildBoxLine(SharpEngineMathUtility.Vector3 Min, SharpEngineMathUtility.Vector3 Max)
		{
			SharpEngineMathUtility.Vector3[] verts = new SharpEngineMathUtility.Vector3[8];
			verts[0] = Min;
			verts[1] = new SharpEngineMathUtility.Vector3(Min.x, Min.y, Max.z); //Z
			verts[2] = new SharpEngineMathUtility.Vector3(Min.x, Max.y, Min.z); //Y
			verts[3] = new SharpEngineMathUtility.Vector3(Max.x, Min.y, Min.z); //X

			verts[7] = Max;
			verts[4] = new SharpEngineMathUtility.Vector3(Max.x, Max.y, Min.z); //Z
			verts[5] = new SharpEngineMathUtility.Vector3(Max.x, Min.y, Max.z); //Y
			verts[6] = new SharpEngineMathUtility.Vector3(Min.x, Max.y, Max.z); //X


			var boxLines = new List<Line>();

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

			return boxLines;

		}

		public static void glLookAt(
			SharpEngineMathUtility.Vector3 eye,
			SharpEngineMathUtility.Vector3 center,
			SharpEngineMathUtility.Vector3 up)
		{
			SharpEngineMathUtility.Vector3 forward = center - eye;

			forward = SharpEngineMathUtility.Vector3.Normalize(forward);

			var upBuf = new SharpEngineMathUtility.Vector3(up.x, up.y, up.z);

			var side = SharpEngineMathUtility.Vector3.Cross(forward, upBuf);

			side = SharpEngineMathUtility.Vector3.Normalize(side);

			upBuf = SharpEngineMathUtility.Vector3.Cross(side, forward);

			var matrix = new float[] {
				Convert.ToSingle (side.x), Convert.ToSingle (up.x), Convert.ToSingle (-forward.x),
				Convert.ToSingle (side.y), Convert.ToSingle (up.y), Convert.ToSingle (-forward.y),
				Convert.ToSingle (side.z), Convert.ToSingle (up.z), Convert.ToSingle (-forward.z)
			};

			GL.MultMatrix(matrix);
			GL.Translate(-eye.x, -eye.y, -eye.z);
		}

		public static void DrawLine(
			SharpEngineMathUtility.Vector3 a,
			SharpEngineMathUtility.Vector3 b)
		{
			GL.Begin(PrimitiveType.Lines);
			GL.Vertex3(a.x, a.y, a.z);
			GL.Vertex3(b.x, b.y, b.z);
			GL.End();
		}

		#region "Private methods"

		/// <summary>
		/// GL draw solid.
		/// </summary>
		/// <param name="solid">Solid.</param>
		/// <param name="GLM_TEXTURE">If set to <c>true</c> GL add texture coordinates.</param>
		/// <param name="GLM_FLAT">If set to <c>true</c> GL add plane normal.</param>
		private static void GLDrawSolid(
			ObjImporter.meshStruct solid,
			SharpEngineMathUtility.Vector3 translate,
			bool GLM_TEXTURE, 
			bool GLM_FLAT,
			bool GLM_SMOOTH)
		{
			int[] indicedata = new int[solid.faceData.Length];
			int[] textureData = new int[solid.faceData.Length];
			int[] normalData = new int[solid.faceData.Length]; 

			GL.Begin (PrimitiveType.Triangles);

			int nTriangle = solid.faceData.Length / 3;

			for (int i = 0; i < nTriangle; i++) 
			{
				int index = i * 3;
				indicedata [index] = (int)solid.faceData[i * 3].x - 1;
				indicedata [index + 1] = (int)solid.faceData[(i * 3) + 1].x - 1;
				indicedata [index + 2] = (int)solid.faceData[(i * 3) + 2].x - 1;

				
				textureData[index] = (int)solid.faceData[i * 3].y - 1;
				textureData[index + 1] = (int)solid.faceData[(i * 3) + 1].y - 1;
				textureData[index + 2] = (int)solid.faceData[(i * 3) + 2].y - 1;
				
				normalData [index] = (int)solid.faceData[i * 3].z - 1;
				normalData [index + 1] = (int)solid.faceData[(i * 3) + 1].z - 1;
				normalData [index + 2] = (int)solid.faceData[(i * 3) + 2].z - 1;

				if (GLM_FLAT) 
				{
					var a = new SharpEngineMathUtility.Vector3(
						solid.vertices[indicedata[index]].x,
						solid.vertices[indicedata[index]].y,
						solid.vertices[indicedata[index]].z);
					
					var b = new SharpEngineMathUtility.Vector3 (
						solid.vertices[indicedata [index + 1]].x, 
						solid.vertices[indicedata [index + 1]].y, 
						solid.vertices[indicedata [index + 1]].z);

					var c = new SharpEngineMathUtility.Vector3 (
						solid.vertices[indicedata [index + 2]].x, 
						solid.vertices[indicedata [index + 2]].y, 
						solid.vertices[indicedata [index + 2]].z);

					SharpEngineMathUtility.Vector3 normal = 
						SharpEngineMathUtility.GeometryUtilities.CalculateNormal (a, b, c);

					GL.Normal3 (normal.x, normal.y, normal.z);	
				}

				if (GLM_SMOOTH)
					GL.Normal3 (
						solid.normals[normalData[index]].x, 
						solid.normals[normalData[index]].y, 
						solid.normals[normalData[index]].z);

				if (GLM_TEXTURE && solid.uv.Length > 0)
					GL.TexCoord2 (
						solid.uv [textureData [index]].x,
						solid.uv [textureData [index]].y);

				GL.Vertex3 (
					solid.vertices[indicedata [index]].x - translate.x, 
					solid.vertices[indicedata [index]].y - translate.y,
					solid.vertices[indicedata [index]].z - translate.z);

				if (GLM_SMOOTH)
					GL.Normal3 (
						solid.normals[normalData[index + 1]].x, 
						solid.normals[normalData[index + 1]].y, 
						solid.normals[normalData[index + 1]].z);

				if (GLM_TEXTURE && solid.uv.Length > 0)
					GL.TexCoord2 (
						solid.uv [textureData [index + 1]].x,
						solid.uv [textureData [index + 1]].y);

				GL.Vertex3 (
					solid.vertices[indicedata [index + 1]].x - translate.x, 
					solid.vertices[indicedata [index + 1]].y - translate.y,
					solid.vertices[indicedata [index + 1]].z - translate.z);

				if (GLM_SMOOTH)
					GL.Normal3 (
						solid.normals[normalData[index + 2]].x, 
						solid.normals[normalData[index + 2]].y, 
						solid.normals[normalData[index + 2]].z);

				if (GLM_TEXTURE && solid.uv.Length > 0)
					GL.TexCoord2 (
						solid.uv [textureData [index + 2]].x,
						solid.uv [textureData [index + 2]].y);

				GL.Vertex3 (
					solid.vertices[indicedata [index + 2]].x - translate.x, 
					solid.vertices[indicedata [index + 2]].y - translate.y,
					solid.vertices[indicedata [index + 2]].z - translate.z);

			}

			GL.End ();
		}

		/// <summary>
		/// GL draw solid.
		/// </summary>
		/// <param name="solid">Solid.</param>
		/// <param name="GLM_TEXTURE">If set to <c>true</c> GL add texture coordinates.</param>
		/// <param name="GLM_FLAT">If set to <c>true</c> GL add plane normal.</param>
		public static void GLDrawSolid(
			SharpEngineMathUtility.Vector3[][] faces,
			SharpEngineMathUtility.Vector3 translate,
			bool GLM_TEXTURE,
			bool GLM_FLAT,
			bool GLM_SMOOTH)
		{
			//int[] indicedata = new int[triIndexes.Length];
			//int[] textureData = new int[triIndexes.Length];
			//int[] normalData = new int[triIndexes.Length];

			GL.Begin(PrimitiveType.Triangles);

			int nTriangle = faces.Length;

			for (int i = 0; i < nTriangle; i++)
			{
				//indicedata[index] = triIndexes;
				//indicedata[index + 1] = (int)solid.faceData[(i * 3) + 1].x - 1;
				//indicedata[index + 2] = (int)solid.faceData[(i * 3) + 2].x - 1;


			   
				//if (GLM_FLAT)
				//{
				//    var a = new SharpEngineMathUtility.Vector3(
				//        solid.vertices[indicedata[index]].x,
				//        solid.vertices[indicedata[index]].y,
				//        solid.vertices[indicedata[index]].z);

				//    var b = new SharpEngineMathUtility.Vector3(
				//        solid.vertices[indicedata[index + 1]].x,
				//        solid.vertices[indicedata[index + 1]].y,
				//        solid.vertices[indicedata[index + 1]].z);

				//    var c = new SharpEngineMathUtility.Vector3(
				//        solid.vertices[indicedata[index + 2]].x,
				//        solid.vertices[indicedata[index + 2]].y,
				//        solid.vertices[indicedata[index + 2]].z);

				//    SharpEngineMathUtility.Vector3 normal =
				//        SharpEngineMathUtility.GeometryUtilities.CalculateNormal(a, b, c);

				//    GL.Normal3(normal.x, normal.y, normal.z);
				//}

				//if (GLM_SMOOTH)
				//    GL.Normal3(
				//        solid.normals[normalData[index]].x,
				//        solid.normals[normalData[index]].y,
				//        solid.normals[normalData[index]].z);

				//if (GLM_TEXTURE && solid.uv.Length > 0)
				//    GL.TexCoord2(
				//        solid.uv[textureData[index]].x,
				//        solid.uv[textureData[index]].y);

				GL.Vertex3(
					faces[i][0].x - translate.x,
					faces[i][0].y - translate.y,
					faces[i][0].z - translate.z);

				//if (GLM_SMOOTH)
				//    GL.Normal3(
				//        solid.normals[normalData[index + 1]].x,
				//        solid.normals[normalData[index + 1]].y,
				//        solid.normals[normalData[index + 1]].z);

				//if (GLM_TEXTURE && solid.uv.Length > 0)
				//    GL.TexCoord2(
				//        solid.uv[textureData[index + 1]].x,
				//        solid.uv[textureData[index + 1]].y);

				GL.Vertex3(
					faces[i][1].x - translate.x,
					faces[i][1].y - translate.y,
					faces[i][1].z - translate.z);

				//if (GLM_SMOOTH)
				//    GL.Normal3(
				//        solid.normals[normalData[index + 2]].x,
				//        solid.normals[normalData[index + 2]].y,
				//        solid.normals[normalData[index + 2]].z);

				//if (GLM_TEXTURE && solid.uv.Length > 0)
				//    GL.TexCoord2(
				//        solid.uv[textureData[index + 2]].x,
				//        solid.uv[textureData[index + 2]].y);

				GL.Vertex3(
					faces[i][2].x - translate.x,
					faces[i][2].y - translate.y,
					faces[i][2].z - translate.z);

			}

			GL.End();
		}

		#endregion

	}
}

