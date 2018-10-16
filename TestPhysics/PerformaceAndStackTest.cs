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
using SharpPhysicsEngine.CollisionEngine;

namespace TestPhysics
{
	public class PerformaceAndStackTest
	{
		#region Fields

		public List<string> ShapeFilename { get; set; }
		public List<string> TextureFilename { get; set; }
		public List<float> ShapeScale { get; set; }
        
        #endregion

		#region Constructor

		public PerformaceAndStackTest()
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

            //physicsEnvironment.RemoveShape(0);

            physicsEnvironment.SetSolverType(SolverType.RedBlackProjectedGaussSeidel);
            physicsEnvironment.CollisionEngineParam.SetBroadPhaseEngine(BroadPhaseEngineType.HierarchicalTree);
            physicsEnvironment.SolverParameters.SetSolverMaxIteration(50);
            physicsEnvironment.SolverParameters.SetSOR(1.0);
            physicsEnvironment.SolverParameters.SetErrorTolerance(1E-8);

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
                    loadObjects[i][j] = LoadObjMesh(ShapeFilename[i], ShapeScale[i], new Vector3d(0.0, 0.0, 0.0), 0.0);
                    if (i > 0)
                        loadObjects[i][j] = LoadObjMesh(ShapeFilename[i], ShapeScale[i], new Vector3d(1.0, 0.0, 0.0), 0.0);
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
			ShapeScale.Add(60);
			TextureFilename.Add("texture/woodbox.bmp");

            GeometryProperties geom0 = GetObjectGeometry(ShapeFilename[0], ShapeScale[0], 0.0);
            var objects0 = new ConvexShape(geom0.VertexPoint, geom0.TriagleIdx, new Vector3d(0.0, -2.0, 0.0), 0.0, true);
            objects0.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 1.0));
            objects0.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects0.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects0.SetRestitutionCoeff(0.1);
            objects0.SetDynamicFrictionCoeff(1.0);
            objects0.SetStaticFrictionCoeff(1.0);
            objects0.ExcludeFromCollisionDetection(false);
            objects0.SetErrorReductionParam(0.3);

            objects.Add(objects0);

            #endregion

            #region Dynamic Objects

            Vector3d shift = new Vector3d(0.0, 2.5, 0.0);
            Vector3d position = new Vector3d(0.0, 4.0, 0.0);
            string objName = "bunny.obj";

            double[] mass = new double[] { 50, 20, 8, 3, 1 };
            
            GeometryProperties geom1 = GetObjectGeometry(objName, 1.0f, 0.0);

            //ShapeGeometry shapeGeometry = new ShapeGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            ShapeGeometry shapeGeometry = new ShapeGeometry(geom1.VertexPoint);

            for (int i = 0; i < 20; i++)
            {
                ShapeFilename.Add(objName);
                ShapeScale.Add(1.0f);
                TextureFilename.Add("texture/woodbox.bmp");

                
                //var objects1 = new ConcaveShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0, false);
                //var objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
                //var objects1 = new ConvexShape(geom1.VertexPoint, position, 1.0);
                var objects1 = new ConvexShape(shapeGeometry, position, 1.0, false);
                objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 1.0));
                objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetRestitutionCoeff(0.1);
                objects1.SetDynamicFrictionCoeff(0.2);
                objects1.SetStaticFrictionCoeff(0.9);
                objects1.ExcludeFromCollisionDetection(false);
                objects1.SetErrorReductionParam(0.3);
                position = position + shift;

                objects.Add(objects1);
            }

            position = new Vector3d(3.0, 4.0, 0.0);

            for (int i = 0; i < 20; i++)
            {
                ShapeFilename.Add(objName);
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");


                //GeometryProperties geom1 = GetObjectGeometry(objName, 1, 0.0);
                //var objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
                var objects1 = new ConvexShape(shapeGeometry, position, 1.0, false);
                objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
                //objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
                objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetRestitutionCoeff(0.1);
                objects1.SetDynamicFrictionCoeff(0.8);
                objects1.SetStaticFrictionCoeff(0.9);
                objects1.ExcludeFromCollisionDetection(false);
                objects1.SetErrorReductionParam(0.3);
                position = position + shift;

                objects.Add(objects1);
            }

            position = new Vector3d(-3.0, 1.7, 0.0);

            for (int i = 0; i < 20; i++)
            {
                ShapeFilename.Add(objName);
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");


                //GeometryProperties geom1 = GetObjectGeometry(objName, 1, 0.0);
                //var objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
                var objects1 = new ConvexShape(shapeGeometry, position, 1.0, false);
                objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
                //objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
                objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetRestitutionCoeff(0.1);
                objects1.SetDynamicFrictionCoeff(0.8);
                objects1.SetStaticFrictionCoeff(0.9);
                objects1.ExcludeFromCollisionDetection(false);
                objects1.SetErrorReductionParam(0.3);
                position = position + shift;

                objects.Add(objects1);
            }

            position = new Vector3d(-3.0, 1.7, 3.0);

            for (int i = 0; i < 20; i++)
            {
                ShapeFilename.Add(objName);
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");


                //GeometryProperties geom1 = GetObjectGeometry(objName, 1, 0.0);
                //var objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
                var objects1 = new ConvexShape(shapeGeometry, position, 1.0, false);
                objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 0.0));
                //objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
                objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetRestitutionCoeff(0.1);
                objects1.SetDynamicFrictionCoeff(0.8);
                objects1.SetStaticFrictionCoeff(0.9);
                objects1.ExcludeFromCollisionDetection(false);
                objects1.SetErrorReductionParam(0.3);
                position = position + shift;

                objects.Add(objects1);
            }
            /*
            position = new Vector3(-3.0, 1.7, 0.0);

            for (int i = 0; i < 5; i++)
            {
                ShapeFilename.Add("cube.obj");
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");

                var objects1 = new RigidCollisionShape();
                objects1.SetMass(1.0);
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
                objects1.SetErrorReductionParam(0.5);
                position = position + shift;

                objects.Add(objects1);
            }


            position = new Vector3(0.0, 1.7, -3.0);

            for (int i = 0; i < 10; i++)
            {
                ShapeFilename.Add("cube.obj");
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");

                var objects1 = new RigidCollisionShape();
                objects1.SetMass(1.0);
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
                objects1.SetErrorReductionParam(0.5);
                position = position + shift;

                objects.Add(objects1);
            }

            position = new Vector3(0.0, 1.7, 3.0);

            for (int i = 0; i < 10; i++)
            {
                ShapeFilename.Add("cube.obj");
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");

                var objects1 = new RigidCollisionShape();
                objects1.SetMass(1.0);
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
                objects1.SetErrorReductionParam(0.5);
                position = position + shift;

                objects.Add(objects1);
            }
            */

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
			float scale,
            double rotate)
		{
			GenericUtility.ObjProperties properties = GenericUtility.GetImportedObjectProperties(fileName, scale);

            RotateObj(ref properties, new Vector3d(1.0, 0.0, 0.0), rotate);

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
			double scale,
            Vector3d versor,
            double angle)
		{
			ObjImporter importer = new ObjImporter();
			ObjImporter.meshStruct mesh = importer.ImportFile(fileName);

			Vector3d[] vertexStartPoint = new Vector3d[mesh.vertices.Length];

			for (int i = 0; i < mesh.vertices.Length; i++)
				vertexStartPoint[i] = mesh.vertices[i];

			OpenGLUtilities.UnitizeObject(ref vertexStartPoint);
			OpenGLUtilities.ScaleObject(ref vertexStartPoint, scale);

            RotateObj(ref vertexStartPoint, versor, angle);
            
            for (int i = 0; i < mesh.vertices.Length; i++)
				mesh.vertices[i] = vertexStartPoint[i];

            

			return mesh;
		}

		private SoftShape BuildSoftBody(
			string fileName,
			double scale,
			Vector3d position)
		{
			GenericUtility.ObjProperties prop = GenericUtility.GetImportedObjectProperties(fileName, scale);

            RotateObj(ref prop, new Vector3d(0.0, 0.0, 1.0), -Math.PI / 4.5);

			return new SoftShape(
				prop.triangleIndex, 
				prop.vertexPoint, 
				position,
                1.0,
                0.2,
                2.0,
                60.0);
		}


        private static void RotateObj(ref GenericUtility.ObjProperties obj, Vector3d versor, double angle)
        {
            for (int i = 0; i < obj.vertexPoint.Length; i++)
            {
                obj.vertexPoint[i] = Vector3d.RotatePoint(obj.vertexPoint[i], versor, angle);
            }
        }

        private static void RotateObj(ref Vector3d[] vertexStartPoint, Vector3d versor, double angle)
        {
            for (int i = 0; i < vertexStartPoint.Length; i++)
            {
                vertexStartPoint[i] = Vector3d.RotatePoint(vertexStartPoint[i], versor, angle);
            }
        }

        #endregion
    }
}

