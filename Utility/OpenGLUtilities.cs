using System;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;
using PhysicsEngineMathUtility;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using ObjLoader;
using ObjLoader.Loader.Loaders;

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
			Bitmap bitmap = new Bitmap (filename);

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
		public static int[] LoadGLObjects(
			LoadResult[] objects,
			PhysicsEngineMathUtility.Vector3[] translate,
			int nObject,
			bool GLM_TEXTURE = false, 
			bool GLM_FLAT = false,
			bool GLM_SMOOTH = false )
		{
			int[] displayList = new int[nObject]; 

			for (int i = 0; i < nObject; i++) 
			{
				displayList [i] = GL.GenLists (1);

				GL.NewList (displayList [i], ListMode.CompileAndExecute);

				OpenGLUtilities.GLDrawSolid (objects[i], translate[i], GLM_TEXTURE, GLM_FLAT, GLM_SMOOTH);

				GL.EndList ();
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
				ObjLoader.Loader.Data.VertexData.Vertex vertex = new ObjLoader.Loader.Data.VertexData.Vertex (
					(obj.Vertices [i].X - cx) * scale,
					(obj.Vertices [i].Y - cy) * scale,
					(obj.Vertices [i].Z - cz) * scale);

				obj.Vertices [i] = vertex;
			}

			return scale;
		}

		public static void ScaleObject(
			ref LoadResult obj, 
			float scale)
		{
			for (int i = 0; i < obj.Vertices.Count; i++) 
			{
				ObjLoader.Loader.Data.VertexData.Vertex vertex = new ObjLoader.Loader.Data.VertexData.Vertex (
					(obj.Vertices [i].X) * scale,
					(obj.Vertices [i].Y) * scale,
					(obj.Vertices [i].Z) * scale);

				obj.Vertices [i] = vertex;
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

		public static void drawSolidCube(float size)
		{
			float[][] n = new float[6] [];
			n [0] = new float[3] { -1.0f, 0.0f, 0.0f };
			n [1] = new float[3] { 0.0f, 1.0f, 0.0f};
			n [2] = new float[3] { 1.0f, 0.0f, 0.0f};
			n [3] = new float[3] { 0.0f, -1.0f, 0.0f};
			n [4] = new float[3] { 0.0f, 0.0f, 1.0f};
			n [5] = new float[3] { 0.0f, 0.0f, -1.0f};

			int[][] faces = new int[6] [];
			faces [0] = new int[4] { 0, 1, 2, 3 };
			faces [1] = new int[4] { 3, 2, 6, 7 };
			faces [2] = new int[4] { 7, 6, 5, 4 };
			faces [3] = new int[4] { 4, 5, 1, 0 };
			faces [4] = new int[4] { 5, 6, 2, 1 };
			faces [5] = new int[4] { 7, 4, 0, 3 };

			float bSize = size / 2.0f;
			float[][] v = new float[8] [];
			v [0] = new float[3] { -bSize, -bSize, -bSize };
			v [1] = new float[3] { -bSize, -bSize, bSize };
			v [2] = new float[3] { -bSize, bSize, bSize };
			v [3] = new float[3] { -bSize, bSize, -bSize };
			v [4] = new float[3] { bSize, -bSize, -bSize };
			v [5] = new float[3] { bSize, -bSize, bSize };
			v [6] = new float[3] { bSize, bSize, bSize };
			v [7] = new float[3] { bSize, bSize, -bSize };

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

		public static void glLookAt(
			PhysicsEngineMathUtility.Vector3 eye,
			PhysicsEngineMathUtility.Vector3 center,
			PhysicsEngineMathUtility.Vector3 up)
		{
			PhysicsEngineMathUtility.Vector3 forward = center - eye;

			forward = PhysicsEngineMathUtility.Vector3.Normalize(forward);

			PhysicsEngineMathUtility.Vector3 upBuf = new PhysicsEngineMathUtility.Vector3(up.x, up.y, up.z);

			PhysicsEngineMathUtility.Vector3 side = PhysicsEngineMathUtility.Vector3.Cross(forward, upBuf);

			side = PhysicsEngineMathUtility.Vector3.Normalize(side);

			upBuf = PhysicsEngineMathUtility.Vector3.Cross(side, forward);

			float[] matrix = new float[] {
				Convert.ToSingle (side.x), Convert.ToSingle (up.x), Convert.ToSingle (-forward.x),
				Convert.ToSingle (side.y), Convert.ToSingle (up.y), Convert.ToSingle (-forward.y),
				Convert.ToSingle (side.z), Convert.ToSingle (up.z), Convert.ToSingle (-forward.z)
			};

			GL.MultMatrix(matrix);
			GL.Translate(-eye.x, -eye.y, -eye.z);
		}

		#region "Private methods"

		/// <summary>
		/// GL draw solid.
		/// </summary>
		/// <param name="solid">Solid.</param>
		/// <param name="GLM_TEXTURE">If set to <c>true</c> GL add texture coordinates.</param>
		/// <param name="GLM_FLAT">If set to <c>true</c> GL add plane normal.</param>
		private static void GLDrawSolid(
			LoadResult solid,
			PhysicsEngineMathUtility.Vector3 translate,
			bool GLM_TEXTURE, 
			bool GLM_FLAT,
			bool GLM_SMOOTH)
		{
			int[] indicedata = new int[solid.Groups [0].Faces.Count * 3];
			int[] textureData = new int[solid.Groups [0].Faces.Count * 3];
			int[] normalData = new int[solid.Groups [0].Faces.Count * 3]; 

			GL.Begin (PrimitiveType.Triangles);

			for (int i = 0; i < solid.Groups [0].Faces.Count; i++) 
			{
				int index = i * 3;
				indicedata [index] = solid.Groups [0].Faces [i] [0].VertexIndex - 1;
				indicedata [index + 1] = solid.Groups [0].Faces [i] [1].VertexIndex - 1;
				indicedata [index + 2] = solid.Groups [0].Faces [i] [2].VertexIndex - 1;

				textureData [index] = solid.Groups [0].Faces [i] [0].TextureIndex - 1;
				textureData [index + 1] = solid.Groups [0].Faces [i] [1].TextureIndex - 1;
				textureData [index + 2] = solid.Groups [0].Faces [i] [2].TextureIndex - 1;

				normalData [index] = solid.Groups [0].Faces [i] [0].NormalIndex - 1;
				normalData [index + 1] = solid.Groups [0].Faces [i] [1].NormalIndex - 1;
				normalData [index + 2] = solid.Groups [0].Faces [i] [2].NormalIndex - 1;

				if (GLM_FLAT) 
				{
					PhysicsEngineMathUtility.Vector3 a = new PhysicsEngineMathUtility.Vector3 (
						solid.Vertices [indicedata [index]].X, 
						solid.Vertices [indicedata [index]].Y, 
						solid.Vertices [indicedata [index]].Z);
					
					PhysicsEngineMathUtility.Vector3 b = new PhysicsEngineMathUtility.Vector3 (
						solid.Vertices [indicedata [index + 1]].X, 
						solid.Vertices [indicedata [index + 1]].Y, 
						solid.Vertices [indicedata [index + 1]].Z);

					PhysicsEngineMathUtility.Vector3 c = new PhysicsEngineMathUtility.Vector3 (
						solid.Vertices [indicedata [index + 2]].X, 
						solid.Vertices [indicedata [index + 2]].Y, 
						solid.Vertices [indicedata [index + 2]].Z);

					PhysicsEngineMathUtility.Vector3 normal = 
						PhysicsEngineMathUtility.GeometryUtilities.CalculateNormal (a, b, c);

					GL.Normal3 (normal.x, normal.y, normal.z);	
				}

				if (GLM_SMOOTH)
					GL.Normal3 (solid.Normals[normalData[index]].X, 
						solid.Normals[normalData[index]].Y, 
						solid.Normals[normalData[index]].Z);

				if (GLM_TEXTURE && solid.Textures.Count > 0)
					GL.TexCoord2 (solid.Textures [textureData [index]].X,
						solid.Textures [textureData [index]].Y);

				GL.Vertex3 (solid.Vertices [indicedata [index]].X - translate.x, 
					solid.Vertices [indicedata [index]].Y - translate.y,
					solid.Vertices [indicedata [index]].Z - translate.z);

				if (GLM_SMOOTH)
					GL.Normal3 (solid.Normals[normalData[index + 1]].X, 
						solid.Normals[normalData[index + 1]].Y, 
						solid.Normals[normalData[index + 1]].Z);

				if (GLM_TEXTURE && solid.Textures.Count > 0)
					GL.TexCoord2 (solid.Textures [textureData [index + 1]].X,
						solid.Textures [textureData [index + 1]].Y);

				GL.Vertex3 (solid.Vertices [indicedata [index + 1]].X - translate.x, 
					solid.Vertices [indicedata [index + 1]].Y - translate.y,
					solid.Vertices [indicedata [index + 1]].Z - translate.z);

				if (GLM_SMOOTH)
					GL.Normal3 (solid.Normals[normalData[index + 2]].X, 
						solid.Normals[normalData[index + 2]].Y, 
						solid.Normals[normalData[index + 2]].Z);

				if (GLM_TEXTURE && solid.Textures.Count > 0)
					GL.TexCoord2 (solid.Textures [textureData [index + 2]].X,
						solid.Textures [textureData [index + 2]].Y);

				GL.Vertex3 (solid.Vertices [indicedata [index + 2]].X - translate.x, 
					solid.Vertices [indicedata [index + 2]].Y - translate.y,
					solid.Vertices [indicedata [index + 2]].Z - translate.z);

			}

			GL.End ();
		}

		#endregion
			
	}
}

