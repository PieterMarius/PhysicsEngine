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
			TextureFilename = new string[nObject-1][];
		}

        #endregion

        #region Public Methods

        public SharpEngine GetPhysicsEnvironmentJoints()
        {
            var physicsEnvironment = new SharpEngine();

            ICollisionShape[] objects = getSphereObject();

            foreach (var obj in objects)
                physicsEnvironment.AddShape(obj);

            ICollisionJoint[] constraints = getConstraint(objects);

            foreach (var item in constraints)
            {
                physicsEnvironment.AddJoint(item);
            }            
            //physicsEnvironment.RemoveShape(0);

            physicsEnvironment.SetSolverType(SolverType.NonLinearConjugateGradient);

            return physicsEnvironment;
        }

        public SharpEngine GetPhysicsEnvironment()
		{
			var physicsEnvironment = new SharpEngine();

			List<ICollisionShape> objects = getSimulationObjects();

			foreach(var obj in objects)
				physicsEnvironment.AddShape(obj);

            //physicsEnvironment.RemoveShape(0);

            physicsEnvironment.SetSolverType(SolverType.RedBlackProjectedGaussSeidel);
            physicsEnvironment.SolverParameters.SetSolverMaxIteration(50);

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

        private ICollisionJoint[] getConstraint(
            ICollisionShape[] shape)
        {
            ICollisionJoint[] constraints = new ICollisionJoint[2];

            constraints[0] = new FixedJoint(
                                shape[1],
                                shape[2],
                                60.0,
                                0.0);

            constraints[1] = new FixedJoint(
                                shape[2],
                                shape[3],
                                60.0,
                                0.0);

            return constraints;
        }

        private ICollisionShape[] getSphereObject()
        {
            ICollisionShape[] objects = new ICollisionShape[4];

            #region Terrain Base

            ShapeFilename[0] = new string[1] { "cube1.obj" };
            ShapeScale[0] = new float[1] { 25 };
            TextureFilename[0] = new string[1] { "texture/woodbox.bmp" };

            objects[0] = new RigidCollisionShape();
            objects[0].SetIsStatic(true);
            objects[0].SetMass(0.0);
            objects[0].SetPosition(new Vector3(0.0, -4.0, 0.0));
            objects[0].SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            GeometryProperties geom0 = GetObjectGeometry(ShapeFilename[0][0], ShapeScale[0][0]);
            objects[0].SetGeometry(geom0.VertexPoint, geom0.TriagleIdx);
            objects[0].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[0].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[0].SetRestitutionCoeff(0.1);
            objects[0].SetDynamicFrictionCoeff(1.0);
            objects[0].SetStaticFrictionCoeff(1.0);
            objects[0].ExcludeFromCollisionDetection(false);
            objects[0].SetErrorReductionParam(30.0);

            #endregion

            ShapeFilename[1] = new string[1] { "sphere.obj" };
            ShapeScale[1] = new float[1] { 1 };
            TextureFilename[1] = new string[1] { "texture/woodbox.bmp" };

            objects[1] = new RigidCollisionShape();
            objects[1].SetMass(1.0);
            objects[1].SetPosition(new Vector3(3.0, 4.0, 7.0));
            objects[1].SetRotationStatus(new Quaternion(1.0, 0.0, 0.0, 0.0));
            GeometryProperties geom1 = GetObjectGeometry(ShapeFilename[1][0], ShapeScale[1][0]);
            objects[1].SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            objects[1].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[1].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[1].SetRestitutionCoeff(0.1);
            objects[1].SetDynamicFrictionCoeff(0.8);
            objects[1].SetStaticFrictionCoeff(0.9);
            objects[1].ExcludeFromCollisionDetection(false);
            objects[1].SetErrorReductionParam(30.0);

            ShapeFilename[2] = new string[1] { "sphere.obj" };
            ShapeScale[2] = new float[1] { 1 };
            TextureFilename[2] = new string[1] { "texture/woodbox.bmp" };

            objects[2] = new RigidCollisionShape();
            objects[2].SetMass(1.0);
            objects[2].SetPosition(new Vector3(0.0, 1.2, 7.0));
            objects[2].SetRotationStatus(new Quaternion(1.0, 0.0, 0.0, 0.0));
            GeometryProperties geom2 = GetObjectGeometry(ShapeFilename[2][0], ShapeScale[2][0]);
            objects[2].SetGeometry(geom2.VertexPoint, geom2.TriagleIdx);
            objects[2].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[2].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[2].SetRestitutionCoeff(0.1);
            objects[2].SetDynamicFrictionCoeff(0.8);
            objects[2].SetStaticFrictionCoeff(0.9);
            objects[2].ExcludeFromCollisionDetection(false);
            objects[2].SetErrorReductionParam(30.0);

            ShapeFilename[3] = new string[1] { "sphere.obj" };
            ShapeScale[3] = new float[1] { 1 };
            TextureFilename[3] = new string[1] { "texture/woodbox.bmp" };

            objects[3] = new RigidCollisionShape();
            objects[3].SetMass(1.0);
            objects[3].SetPosition(new Vector3(-3.0, 1.2, 7.0));
            objects[3].SetRotationStatus(new Quaternion(1.0, 0.0, 0.0, 0.0));
            GeometryProperties geom3 = GetObjectGeometry(ShapeFilename[3][0], ShapeScale[3][0]);
            objects[3].SetGeometry(geom3.VertexPoint, geom3.TriagleIdx);
            objects[3].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[3].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects[3].SetRestitutionCoeff(0.5);
            objects[3].SetDynamicFrictionCoeff(0.8);
            objects[3].SetStaticFrictionCoeff(0.9);
            objects[3].ExcludeFromCollisionDetection(false);
            objects[3].SetErrorReductionParam(30.0);

            return objects;
        }

        private List<ICollisionShape> getSimulationObjects()
		{
			List<ICollisionShape> objects = new List<ICollisionShape>();
			
			#region Terrain Base

			ShapeFilename[0] = new string[1] { "cube1.obj" };
			ShapeScale[0] = new float[1] { 25 };
			TextureFilename[0] = new string[1] { "texture/woodbox.bmp" };

			var objects0 = new RigidCollisionShape();
            objects0.SetIsStatic(true);
            objects0.SetMass(0.0);
            objects0.SetPosition(new Vector3(0.0, -4.0, 0.0));
            objects0.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            GeometryProperties geom0= GetObjectGeometry(ShapeFilename[0][0], ShapeScale[0][0]);
            objects0.SetGeometry(geom0.VertexPoint, geom0.TriagleIdx);
            objects0.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects0.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects0.SetRestitutionCoeff(0.1);
            objects0.SetDynamicFrictionCoeff(1.0);
            objects0.SetStaticFrictionCoeff(1.0);
            objects0.ExcludeFromCollisionDetection(false);
            objects0.SetErrorReductionParam(1.0);

            objects.Add(objects0);

			#endregion

			#region Dynamic Objects

			ShapeFilename[1] = new string[1] { "cube1.obj" };
			ShapeScale[1] = new float[1] { 1 };
			TextureFilename[1] = new string[1] { "texture/woodbox.bmp" };

			var objects1 = new RigidCollisionShape();
            objects1.SetMass(1.0);
            objects1.SetPosition(new Vector3(0.0, 1.2, 9.5));
            objects1.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            GeometryProperties geom1 = GetObjectGeometry(ShapeFilename[1][0], ShapeScale[1][0]);
            objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
            objects1.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3(0.0, 2.0, 0.0));
            objects1.SetRestitutionCoeff(0.5);
            objects1.SetDynamicFrictionCoeff(0.8);
            objects1.SetStaticFrictionCoeff(0.9);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetErrorReductionParam(30.0);

            objects.Add(objects1);

			ShapeFilename[2] = new string[1] { "cube1.obj" };
			ShapeScale[2] = new float[1] { 1 };
			TextureFilename[2] = new string[1] { "texture/woodbox.bmp" };

			var objects2 = new RigidCollisionShape();
            objects2.SetMass(1.0);
            objects2.SetPosition(new Vector3(0.0, 1.2, 7.0));
            objects2.SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            GeometryProperties geom2 = GetObjectGeometry(ShapeFilename[2][0], ShapeScale[2][0]);
            objects2.SetGeometry(geom2.VertexPoint, geom2.TriagleIdx);
            objects2.SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
            objects2.SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
            objects2.SetRestitutionCoeff(0.1);
            objects2.SetDynamicFrictionCoeff(0.8);
            objects2.SetStaticFrictionCoeff(0.9);
            objects2.ExcludeFromCollisionDetection(false);
            objects2.SetErrorReductionParam(30.0);

            objects.Add(objects2);

            //TextureFilename[3] = new string[1] { "texture/woodbox.bmp" };
            //TODO rimuovere
            string softObject = "sphere.obj";
            ShapeFilename[3] = new string[1] { softObject };
			ShapeScale[3] = new float[1] { 1 };
            
			var objects3 = BuildSoftBody(softObject, 1, new Vector3(0.0, 0.0, 0.0));
            objects3.SetStaticFrictionCoeff(0.5);
            objects3.SetDynamicFrictionCoeff(0.5);
            objects3.SetRestitutionCoeff(1.0);
            objects3.SetErrorReductionParam(1.0);

            objects.Add(objects3);

            #endregion

            return objects;
		}

		private void getSimulationConstraints()
		{
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

		private SoftCollisionShape BuildSoftBody(
			string fileName,
			double scale,
			Vector3 position)
		{
			GenericUtility.ObjProperties prop = GenericUtility.GetImportedObjectProperties(fileName, scale);

            //RotateObj(ref prop, new Vector3(0.0, 0.0, 1.0), -Math.PI / 4.5);

			return new SoftCollisionShape(
				prop.triangleIndex, 
				prop.vertexPoint, 
				position,
                1.0,
                0.2,
                0.7,
                8.0,
                0.1,
                0.1);
		}


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

