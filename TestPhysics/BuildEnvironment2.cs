using System.IO;
using SharpPhysicsEngine;
using ObjLoader.Loader.Loaders;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using Utility;
using SharpPhysicsEngine.LCPSolver;
using System;
using System.Collections.Generic;
using SharpPhysicsEngine.Wrapper;
using SharpPhysicsEngine.Wrapper.Joint;

namespace TestPhysics
{
	public class BuildEnvironment2
	{
		#region Fields

		public List<string> ShapeFilename { get; set; }
		public List<string> TextureFilename { get; set; }
		public List<float> ShapeScale { get; set; }
        
        #endregion

		#region Constructor

		public BuildEnvironment2()
		{
			ShapeFilename = new List<string>();
			ShapeScale = new List<float>();
			TextureFilename = new List<string>();
		}

        #endregion

        #region Public Methods
        
        public SharpEngine GetPhysicsEnvironment()
		{
			var physicsEnvironment = new SharpEngine();

			List<ICollisionShape> objects = GetSimulationObjects();

			foreach(var obj in objects)
				physicsEnvironment.AddShape(obj);

            ICollisionJoint[] constraints = getConstraint(objects);

            foreach (var item in constraints)
            {
                physicsEnvironment.AddJoint(item);
            }

            //physicsEnvironment.RemoveShape(0);

            physicsEnvironment.SetSolver(SolverType.ProjectedConjugateGradient);

            return physicsEnvironment;
		}

        public int[][] LoadTexture()
		{
			int[][] textureID = new int[TextureFilename.Count][];

			for (int i = 0; i < TextureFilename.Count; i++)
			{
				textureID[i] = new int[1];

				for (int j = 0; j < 1; j++)
					textureID[i][j] = OpenGLUtilities.LoadTexture(TextureFilename[i]);
			}
			return textureID;
		}

        public int[][] GetOpenGLEnvironment()
        {
            ObjImporter.meshStruct[][] loadObjects = new ObjImporter.meshStruct[ShapeFilename.Count][];

            for (int i = 0; i < ShapeFilename.Count; i++)
            {
                loadObjects[i] = new ObjImporter.meshStruct[1];

                for (int j = 0; j < 1; j++)
                {
                    loadObjects[i][j] = LoadObjMesh(ShapeFilename[i], ShapeScale[i]);
                }
            }

            return OpenGLUtilities.LoadGLObjects(
                loadObjects,
                ShapeFilename.Count,
                true,
                false,
                true);
        }


        #endregion

        #region Private Methods



        private List<ICollisionShape> GetSimulationObjects()
		{
			List<ICollisionShape> objects = new List<ICollisionShape>();

            #region Terrain Base

            ShapeFilename.Add("cube1.obj");
			ShapeScale.Add(25);
			TextureFilename.Add("texture/woodbox.bmp");

			var objects0 = new StaticCollisionShape();
            objects0.SetMass(0.0);
            objects0.SetPosition(new Vector3(0.0, -4.0, 0.0));
            objects0.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            GeometryProperties geom0= GetObjectGeometry(ShapeFilename[0], ShapeScale[0]);
            objects0.SetGeometry(geom0.VertexPoint, geom0.TriagleIdx);
            objects0.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects0.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects0.SetRestitutionCoeff(0.1);
            objects0.SetDynamicFrictionCoeff(1.0);
            objects0.SetStaticFrictionCoeff(1.0);
            objects0.ExcludeFromCollisionDetection(false);
            objects0.SetRestoreCoeff(60.0);

            objects.Add(objects0);

            #endregion

            #region Dynamic Objects
                        
            Vector3 position = new Vector3(0.0, 5.6, 0.0);
                        
            ShapeFilename.Add("cube1.obj");
            ShapeScale.Add(2);
            TextureFilename.Add("texture/woodbox.bmp");
                
            var objects1 = new RigidCollisionShape();
            objects1.SetMass(40.0);
            objects1.SetPosition(position);
            objects1.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            GeometryProperties geom1 = GetObjectGeometry("cube.obj", 1);
            objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            objects1.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetRestoreCoeff(30.0);
            objects.Add(objects1);

            position = new Vector3(-5-0, 4.6, -1.5);

            ShapeFilename.Add("cube.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            objects1 = new RigidCollisionShape();
            objects1.SetMass(1.0);
            objects1.SetPosition(position);
            objects1.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            geom1 = GetObjectGeometry("cube.obj", 1);
            objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            objects1.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetRestoreCoeff(30.0);
            objects.Add(objects1);
            
            position = new Vector3(5.0, 4.6, -1.5);
                        
            ShapeFilename.Add("cube.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            objects1 = new RigidCollisionShape();
            objects1.SetMass(1.0);
            objects1.SetPosition(position);
            objects1.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            geom1 = GetObjectGeometry("cube.obj", 1);
            objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            objects1.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetRestoreCoeff(30.0);
                
            objects.Add(objects1);

            position = new Vector3(5.0, 4.6, 1.5);

            ShapeFilename.Add("cube.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            objects1 = new RigidCollisionShape();
            objects1.SetMass(1.0);
            objects1.SetPosition(position);
            objects1.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            geom1 = GetObjectGeometry("cube.obj", 1);
            objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            objects1.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetRestoreCoeff(30.0);

            objects.Add(objects1);

            position = new Vector3(-5.0, 4.6, 1.5);

            ShapeFilename.Add("cube.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            objects1 = new RigidCollisionShape();
            objects1.SetMass(1.0);
            objects1.SetPosition(position);
            objects1.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            geom1 = GetObjectGeometry("cube.obj", 1);
            objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            objects1.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetRestoreCoeff(30.0);

            objects.Add(objects1);
            
            //TextureFilename[3] = new string[1] { "texture/woodbox.bmp" };
            //TODO rimuovere
            //<ShapeFilename[3] = new string[1] { "torus.obj" };
            //ShapeScale[3] = new float[1] { 1 };

            //var objects3 = BuildSoftBody("torus.obj", 1, new Vector3(0.0, -1.5, 0.0));
            //         objects3.SetStaticFrictionCoeff(0.5);
            //         objects3.SetDynamicFrictionCoeff(0.5);
            //         objects3.SetRestitutionCoeff(0.5);
            //         objects3.SetRestoreCoeff(60.0);

            //         objects.Add(objects3);

            #endregion

            return objects;
		}

        private ICollisionJoint[] getConstraint(
            List<ICollisionShape> shape)
        {
            ICollisionJoint[] constraints = new ICollisionJoint[4];

            constraints[0] = new Hinge2Joint(
                                shape[1],
                                shape[2],
                                new Vector3(-3.0, -1.0, -1.5),
                                new Vector3(0.0, 1.0, 0.0),
                                new Vector3(1.0, 0.0, 0.0),
                                13.2,
                                1.0,
                                0.0);

            constraints[0].SetAxis1AngularLimit(0.0, 0.0);

            constraints[1] = new Hinge2Joint(
                                shape[1],
                                shape[4],
                                new Vector3(3.0, -1.0, 1.5),
                                new Vector3(0.0, 1.0, 0.0),
                                new Vector3(1.0, 0.0, 0.0),
                                13.2,
                                1.0,
                                0.0);

            constraints[1].SetAxis1AngularLimit(-0.78539816339, 0.78539816339);

            constraints[2] = new Hinge2Joint(
                                shape[1],
                                shape[3],
                                new Vector3(3.0, -1.0, -1.5),
                                new Vector3(0.0, 1.0, 0.0),
                                new Vector3(1.0, 0.0, 0.0),
                                13.2,
                                1.0,
                                0.0);

            constraints[2].SetAxis1AngularLimit(0.0, 0.0);

            constraints[3] = new Hinge2Joint(
                                shape[1],
                                shape[5],
                                new Vector3(-3.0, -1.0, 1.5),
                                new Vector3(0.0, 1.0, 0.0),
                                new Vector3(1.0, 0.0, 0.0),
                                13.2,
                                1.0,
                                0.0);

            constraints[3].SetAxis1AngularLimit(-0.78539816339, 0.78539816339);

            return constraints;
        }


        public class GeometryProperties
        {
            public Vector3[] VertexPoint { get; private set; }
            public TriangleIndexes[] TriagleIdx { get; private set; }

            public GeometryProperties(
                Vector3[] vertexPoint,
                TriangleIndexes[] triangleIndexes)
            {
                VertexPoint = vertexPoint;
                TriagleIdx = triangleIndexes;
            }
        }
        
		public static GeometryProperties GetObjectGeometry(
			string fileName,
			float scale)
		{
			GenericUtility.ObjProperties properties = GenericUtility.GetImportedObjectProperties(fileName, scale);

            return new GeometryProperties(
                properties.vertexPoint,
                properties.triangleIndex);
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

		//private SoftCollisionShape BuildSoftBody(
		//	string fileName,
		//	double scale,
		//	Vector3 position)
		//{
		//	GenericUtility.ObjProperties prop = GenericUtility.GetImportedObjectProperties(fileName, scale);

  //          RotateObj(ref prop, new Vector3(0.0, 0.0, 1.0), -Math.PI / 4.5);

		//	return new SoftCollisionShape(
		//		prop.triangleIndex, 
		//		prop.vertexPoint, 
		//		position,
  //              0.2,
  //              2.0,
  //              60.0);
		//}


        private void RotateObj(ref GenericUtility.ObjProperties obj, Vector3 versor, double angle)
        {
            for (int i = 0; i < obj.vertexPoint.Length; i++)
            {
                obj.vertexPoint[i] = Vector3.RotatePoint(obj.vertexPoint[i], versor, angle);
            }
        }

        #endregion
    }
}

