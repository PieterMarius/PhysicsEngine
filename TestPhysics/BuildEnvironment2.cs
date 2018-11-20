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

			List<ICollisionShape> objects = BuildBaseAndCarShapes();

			foreach(var obj in objects)
				physicsEnvironment.AddShape(obj);

            ICollisionJoint[] constraints = GetCarConstraints(objects);

            foreach (var item in constraints)
            {
                physicsEnvironment.AddJoint(item);
            }

            var bridge = BuildBridge();

            var bridgeConstraints = GetBridgeConstraints(bridge);

            foreach (var item in bridge)
            {
                physicsEnvironment.AddShape(item);
            }

            foreach (var item in bridgeConstraints)
            {
                physicsEnvironment.AddJoint(item);
            }

            //physicsEnvironment.RemoveShape(0);

            
            physicsEnvironment.EngineParameters.SetFrictionDirection(2);
            physicsEnvironment.SolverParameters.SetSolverMaxIteration(200);
            physicsEnvironment.SolverParameters.SetSOR(1.0);
            physicsEnvironment.SolverParameters.SetErrorTolerance(1E-10);
            physicsEnvironment.SetSolverType(SolverType.RedBlackProjectedGaussSeidel);

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



        private List<ICollisionShape> BuildBaseAndCarShapes()
		{
			List<ICollisionShape> objects = new List<ICollisionShape>();

            #region Terrain Base

            ShapeFilename.Add("cube1.obj");
			ShapeScale.Add(60);
			TextureFilename.Add("texture/woodbox.bmp");

            GeometryProperties geom0 = GetObjectGeometry(ShapeFilename[0], ShapeScale[0]);
            var objects0 = new ConvexShape(geom0.VertexPoint, geom0.TriagleIdx, new Vector3d(0.0, -4.0, 0.0), 0.0, true);
            objects0.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects0.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects0.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects0.SetRestitutionCoeff(0.1);
            objects0.SetDynamicFrictionCoeff(1.0);
            objects0.SetStaticFrictionCoeff(1.0);
            objects0.ExcludeFromCollisionDetection(false);
            objects0.SetErrorReductionParam(0.7);

            objects.Add(objects0);

            #endregion

            #region Dynamic Objects
                        
            Vector3d position = new Vector3d(0.0, 5.6, 0.0);
                        
            ShapeFilename.Add("cube1.obj");
            ShapeScale.Add(2);
            TextureFilename.Add("texture/woodbox.bmp");

            GeometryProperties geom1 = GetObjectGeometry("cube1.obj", 2);
            var objects_0 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 50.0, false);
                      
            objects_0.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects_0.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects_0.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects_0.SetRestitutionCoeff(0.1);
            objects_0.SetDynamicFrictionCoeff(0.8);
            objects_0.SetStaticFrictionCoeff(0.9);
            objects_0.ExcludeFromCollisionDetection(false);
            objects_0.SetErrorReductionParam(0.3);
            objects.Add(objects_0);
                                    
            position = new Vector3d(-3.5, 4.6, -1.9);
            
            ShapeFilename.Add("wheel.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            geom1 = GetObjectGeometry("wheel.obj", 1);
            var objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);

            objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetErrorReductionParam(0.3);
            objects.Add(objects1);
            
            position = new Vector3d(3.5, 4.6, -1.9);
                        
            ShapeFilename.Add("wheel.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            geom1 = GetObjectGeometry("wheel.obj", 1);
            objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
            objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetErrorReductionParam(0.3);
                
            objects.Add(objects1);

            position = new Vector3d(3.5, 4.6, 1.9);

            ShapeFilename.Add("wheel.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            geom1 = GetObjectGeometry("wheel.obj", 1);
            objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
            objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetErrorReductionParam(0.3);

            objects.Add(objects1);

            position = new Vector3d(-3.5, 4.6, 1.9);

            ShapeFilename.Add("wheel.obj");
            ShapeScale.Add(1);
            TextureFilename.Add("texture/woodbox.bmp");

            geom1 = GetObjectGeometry("wheel.obj", 1);
            objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
            objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.1);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetErrorReductionParam(0.3);

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

        private List<ICollisionShape> BuildBridge()
        {
            List<ICollisionShape> objects = new List<ICollisionShape>();

            Vector3d position = new Vector3d(0.0, 0.0, 12.5);

            ShapeFilename.Add("cube.obj");
            ShapeScale.Add(1.5f);
            TextureFilename.Add("texture/woodbox.bmp");

            GeometryProperties geom1 = GetObjectGeometry("cube.obj", 1.5f);
            var objects_0 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 0.0, true);

            objects_0.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects_0.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects_0.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects_0.SetRestitutionCoeff(0.1);
            objects_0.SetDynamicFrictionCoeff(0.8);
            objects_0.SetStaticFrictionCoeff(0.9);
            objects_0.ExcludeFromCollisionDetection(false);
            objects_0.SetErrorReductionParam(0.3);
            objects.Add(objects_0);

            position = new Vector3d(0.0, 1.2, 9.5);

            for (int i = 0; i < 9; i++)
            {
                ShapeFilename.Add("cube1.obj");
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");

                var geom = GetObjectGeometry("cube1.obj", 1);
                var objects_1 = new ConvexShape(geom.VertexPoint, geom.TriagleIdx, position, 1.0, false);

                objects_1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
                objects_1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects_1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects_1.SetRestitutionCoeff(0.1);
                objects_1.SetDynamicFrictionCoeff(0.8);
                objects_1.SetStaticFrictionCoeff(0.9);
                objects_1.ExcludeFromCollisionDetection(false);
                objects_1.SetErrorReductionParam(0.3);
                objects.Add(objects_1);

                position = position - new Vector3d(0.0, 0.0, 2.5);
            }

            position = new Vector3d(0.0, 0.0, -13.5);
            ShapeFilename.Add("cube.obj");
            ShapeScale.Add(1.5f);
            TextureFilename.Add("texture/woodbox.bmp");

            geom1 = GetObjectGeometry("cube.obj", 1.5f);
            objects_0 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 0.0, true);

            objects_0.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
            objects_0.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects_0.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects_0.SetRestitutionCoeff(0.1);
            objects_0.SetDynamicFrictionCoeff(0.8);
            objects_0.SetStaticFrictionCoeff(0.9);
            objects_0.ExcludeFromCollisionDetection(false);
            objects_0.SetErrorReductionParam(0.3);
            objects.Add(objects_0);


            return objects;
        }

        private ICollisionJoint[] GetBridgeConstraints(List<ICollisionShape> shape)
        {
            ICollisionJoint[] constraints = new ICollisionJoint[30];

            int idx = 0;
            double zValue = -1.8;
            double yValue = 1.2;
            for (int i = 0; i < 10; i++)
            {
                
                constraints[idx] = new BallAndSocketJoint(
                    shape[i],
                    shape[i + 1],
                    new Vector3d(0.5, yValue, zValue),
                    0.85,
                    0.00016);
                idx++;

                constraints[idx] = new BallAndSocketJoint(
                    shape[i],
                    shape[i + 1],
                    new Vector3d(-0.5, yValue, zValue),
                    0.85,
                    0.00016);
                idx++;
                                
                constraints[idx] = new AngularJoint(
                    shape[i],
                    shape[i + 1],
                    new Vector3d(-0.5, yValue, zValue),
                    new Vector3d(1.0, 0.0, 0.0),
                    new Vector3d(0.0, 1.0, 0.0),
                    0.16,
                    0.008,
                    0.008);
                idx++;

                zValue = -1.25;
                yValue = 0.0;
            }

            return constraints;
        }

        private ICollisionJoint[] GetCarConstraints(List<ICollisionShape> shape)
        {
            ICollisionJoint[] constraints = new ICollisionJoint[4];

            //Wheels Joints

            constraints[0] = new Hinge2Joint(
                                shape[1],
                                shape[2],
                                shape[3],
                                new Vector3d(-3.5, -1.0, -1.9),
                                new Vector3d(0.0, 1.0, 0.0),
                                new Vector3d(1.0, 0.0, 0.0),
                                1.0,
                                0.5,
                                0.01);
            
            constraints[0].SetAxis1AngularLimit(0.0, 0.0);
                        
            constraints[1] = new Hinge2Joint(
                                shape[1],
                                shape[4],
                                shape[5],
                                new Vector3d(3.5, -1.0, 1.9),
                                new Vector3d(0.0, 1.0, 0.0),
                                new Vector3d(1.0, 0.0, 0.0),
                                1.0,
                                0.5,
                                0.01);
            
            constraints[1].SetAxis1AngularLimit(-0.78539816339, 0.78539816339);
            
            constraints[2] = new Hinge2Joint(
                                shape[1],
                                shape[3],
                                shape[2],
                                new Vector3d(3.5, -1.0, -1.9),
                                new Vector3d(0.0, 1.0, 0.0),
                                new Vector3d(1.0, 0.0, 0.0),
                                1.0,
                                0.5,
                                0.01);
            
            constraints[2].SetAxis1AngularLimit(0.0, 0.0);
            
            constraints[3] = new Hinge2Joint(
                                shape[1],
                                shape[5],
                                shape[4],
                                new Vector3d(-3.5, -1.0, 1.9),
                                new Vector3d(0.0, 1.0, 0.0),
                                new Vector3d(1.0, 0.0, 0.0),
                                1.0,
                                0.5,
                                0.01);

            constraints[3].SetAxis1AngularLimit(-0.78539816339, 0.78539816339);
            
            return constraints;
        }


        public class GeometryProperties
        {
            public Vector3d[] VertexPoint { get; private set; }
            public int[][] TriagleIdx { get; private set; }

            public GeometryProperties(
                Vector3d[] vertexPoint,
                int[][] triangleIndexes)
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

			Vector3d[] vertexStartPoint = new Vector3d[mesh.vertices.Length];

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


        private void RotateObj(ref GenericUtility.ObjProperties obj, Vector3d versor, double angle)
        {
            for (int i = 0; i < obj.vertexPoint.Length; i++)
            {
                obj.vertexPoint[i] = Vector3d.RotatePoint(obj.vertexPoint[i], versor, angle);
            }
        }

        #endregion
    }
}

