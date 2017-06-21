using System.IO;
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine;
using ObjLoader.Loader.Loaders;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using Utility;
using System;

namespace TestPhysics
{
	public class BuildEnvironment
	{
		#region Fields

		public string[][] ShapeFilename { get; set; }
		public string[][] TextureFilename { get; set; }
		public float[][] ShapeScale { get; set; }

		private int nObject;

		#endregion

		#region Constructor

		public BuildEnvironment()
		{
			nObject = 4;
			ShapeFilename = new string[nObject][];
			ShapeScale = new float[nObject][];
			TextureFilename = new string[nObject][];
		}

		#endregion

		#region Public Methods

		public SharpEngine GetPhysicsEnvironment()
		{
			var physicsEnvironment = new SharpEngine();

			IShape[] objects = getSimulationObjects();

			foreach(var obj in objects)
				physicsEnvironment.AddShape(obj);
			
			//physicsEnvironment.RemoveShape(0);

			return physicsEnvironment;
		}

		public int[][] GetOpenGLEnvironment()
		{
			ObjImporter.meshStruct[][] loadObjects = new ObjImporter.meshStruct[ShapeFilename.Length][];
			
			for (int i = 0; i < ShapeFilename.Length; i++)
			{
				loadObjects[i] = new ObjImporter.meshStruct[ShapeFilename[i].Length];

				for (int j = 0; j < ShapeFilename[i].Length; j++)
				{
					loadObjects[i][j] = LoadObjMesh(ShapeFilename[i][j], ShapeScale[i][j]);
				}
			}
			
			return OpenGLUtilities.LoadGLObjects(
				loadObjects,
				ShapeFilename.Length,
				true,
				false,
				true);
		}

		public int[][] LoadTexture()
		{
			int[][] textureID = new int[TextureFilename.Length][];

			for (int i = 0; i < TextureFilename.Length; i++)
			{
				textureID[i] = new int[TextureFilename[i].Length];

				for (int j = 0; j < TextureFilename[i].Length; j++)
					textureID[i][j] = OpenGLUtilities.LoadTexture(TextureFilename[i][j]);
			}
			return textureID;
		}


		#endregion

		#region Private Methods

		private IShape[] getSimulationObjects()
		{
			IShape[] objects = new IShape[nObject];
			
			#region Terrain Base

			ShapeFilename[0] = new string[1] { "cube1.obj" };
			ShapeScale[0] = new float[1] { 25 };
			TextureFilename[0] = new string[1] { "texture/woodbox.bmp" };

			objects[0] = new ConvexShape(ObjectType.StaticBody);
			objects[0].SetMass(0.0);
			objects[0].SetPosition(new Vector3(0.0, -4.0, 0.0));
			objects[0].SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
			((ConvexShape)objects[0]).SetObjectGeometry(GetObjectGeometry(objects[0], ShapeFilename[0][0], ShapeScale[0][0], ObjectGeometryType.ConvexBody));
			objects[0].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[0].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[0].SetRestitutionCoeff(0.1);
			objects[0].SetDynamicFrictionCoeff(1.0);
			objects[0].SetStaticFrictionCoeff(1.0);
			objects[0].SetExcludeFromCollisionDetection(false);
			objects[0].SetRestoreCoeff(30.0);

			#endregion

			#region Dynamic Objects

			ShapeFilename[1] = new string[1] { "cube1.obj" };
			ShapeScale[1] = new float[1] { 1 };
			TextureFilename[1] = new string[1] { "texture/woodbox.bmp" };

			objects[1] = new ConvexShape(ObjectType.RigidBody);
			objects[1].SetMass(1.0);
			objects[1].SetPosition(new Vector3(0.0, 1.2, 9.5));
			objects[1].SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
			((ConvexShape)objects[1]).SetObjectGeometry(GetObjectGeometry(objects[1], ShapeFilename[1][0], ShapeScale[1][0], ObjectGeometryType.ConvexBody));
			objects[1].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[1].SetAngularVelocity(new Vector3(0.0, 2.0, 0.0));
			objects[1].SetRestitutionCoeff(0.1);
			objects[1].SetDynamicFrictionCoeff(0.8);
			objects[1].SetStaticFrictionCoeff(0.9);
			objects[1].SetExcludeFromCollisionDetection(false);
			objects[1].SetRestoreCoeff(30.0);

			ShapeFilename[2] = new string[1] { "cube1.obj" };
			ShapeScale[2] = new float[1] { 1 };
			TextureFilename[2] = new string[1] { "texture/woodbox.bmp" };

			objects[2] = new ConvexShape(ObjectType.RigidBody);
			objects[2].SetMass(1.0);
			objects[2].SetPosition(new Vector3(0.0, 1.2, 7.0));
			objects[2].SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
			((ConvexShape)objects[2]).SetObjectGeometry(GetObjectGeometry(objects[2], ShapeFilename[2][0], ShapeScale[2][0], ObjectGeometryType.ConvexBody));
			objects[2].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[2].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[2].SetRestitutionCoeff(0.1);
			objects[2].SetDynamicFrictionCoeff(0.8);
			objects[2].SetStaticFrictionCoeff(0.9);
			objects[2].SetExcludeFromCollisionDetection(false);
			objects[2].SetRestoreCoeff(30.0);

			TextureFilename[3] = new string[1] { "texture/woodbox.bmp" };
			//TODO rimuovere
			ShapeFilename[3] = new string[1] { "bunny.obj" };
			ShapeScale[3] = new float[1] { 1 };
			objects[3] = BuildSoftBody("bunny.obj", 1, 0.5, new Vector3(0.0, 0.0, 0.0));

			#endregion

			return objects;
		}

		private void getSimulationConstraints()
		{
		}

		public static IGeometry GetObjectGeometry(
			IShape shape,
			string fileName,
			float scale,
			ObjectGeometryType geometryType)
		{
			GenericUtility.ObjProperties properties = GenericUtility.GetImportedObjectProperties(fileName, scale);

			return new Geometry(
				shape,
				properties.vertexPoint,
				properties.triangleIndex,
				geometryType,
				true);
		}

		private LoadResult LoadObjSolid(
			string fileName,
			float scale)
		{
			var objLoaderFactory = new ObjLoaderFactory();
			var objLoader = objLoaderFactory.Create();
			var fileStream = new FileStream(fileName, FileMode.OpenOrCreate);
			LoadResult solid = objLoader.Load(fileStream);
			fileStream.Close();

			OpenGLUtilities.UnitizeObject(ref solid);
			OpenGLUtilities.ScaleObject(ref solid, scale);

			return solid;
		}

		private ObjImporter.meshStruct LoadObjMesh(
			string fileName,
			double scale)
		{
			ObjImporter importer = new ObjImporter();
			ObjImporter.meshStruct mesh = importer.ImportFile(fileName);

			Vector3[] vertexStartPoint = new Vector3[mesh.vertices.Length];

			for (int i = 0; i < mesh.vertices.Length; i++)
				vertexStartPoint[i] = mesh.vertices[i];

			OpenGLUtilities.UnitizeObject(ref vertexStartPoint);
			OpenGLUtilities.ScaleObject(ref vertexStartPoint, scale);

			for (int i = 0; i < mesh.vertices.Length; i++)
				mesh.vertices[i] = vertexStartPoint[i];

			return mesh;
		}

		private SoftShape BuildSoftBody(
			string fileName,
			double scale,
			double diameter,
			Vector3 position)
		{
			GenericUtility.ObjProperties prop = GenericUtility.GetImportedObjectProperties(fileName, scale);

			return new SoftShape(
				prop.triangleIndex, 
				prop.vertexPoint, 
				diameter, 
				position);
		}

		#endregion
	}
}

