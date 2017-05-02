using System.IO;
using System.Linq;
using CollisionEngine;
using LCPSolver;
using SharpPhysicsEngine;
using ObjLoader.Loader.Loaders;
using PhysicsEngineMathUtility;
using ShapeDefinition;
using Utility;
using System;

namespace TestPhysics
{
	public class BuildEnvironment
	{
		#region COnstructor

		public BuildEnvironment()
		{
			
		}

		#endregion

		#region Public Methods

		public SharpEngine GetPhysicsEnvironment()
		{
			var simulationParam = new PhysicsEngineParameters();
			var solverParameters = new SolverParameters();
			var collisionEngineParam = new CollisionEngineParameters();

            var physicsEnvironment = new SharpEngine(
                                                simulationParam,
                                                collisionEngineParam,
                                                solverParameters);

			physicsEnvironment.AddShape(getSimulationObjects()[0]);
			physicsEnvironment.RemoveShape(0);

			return physicsEnvironment;
		}

		#endregion

		#region Private Methods

		private IShape[] getSimulationObjects()
		{
			CompoundShape[] objects = new CompoundShape[1];

			#region Terrain Base

			objects[0].SetPartialMass(new double[] { 1000.0 });
            objects[0].SetPosition(new Vector3(0.0, -4.0, 0.0));
            objects[0].SetRotationStatus(new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
            objects[0].SetObjectGeometry(new Geometry[] { GetObjectGeometry(objects[0], "cube.obj", 25) });

            objects[0].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[0].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[0].SetRestitutionCoeff(0.1);
			objects[0].SetDynamicFrictionCoeff(1.0);
			objects[0].SetStaticFrictionCoeff(1.0);
			objects[0].SetExcludeFromCollisionDetection(false);

            Vector3[] vertexPosition = Array.ConvertAll(objects[0].ObjectGeometry[0].VertexPosition,
                                        item => item.Vertex);

            var inertiaTensor = new InertiaTensor(
                vertexPosition,
				objects[0].ObjectGeometry[0].Triangle,
				objects[0].Mass);

			//Traslo per normalizzare l'oggetto rispetto al suo centro di massa
			for (int j = 0; j < objects[0].ObjectGeometry[0].VertexPosition.Length; j++)
			{
				objects[0].ObjectGeometry[0].SetVertexPosition(
                    vertexPosition[j] - inertiaTensor.GetMassCenter(),
					j);
			}

			var inertiaTensor1 = new InertiaTensor(
                vertexPosition,
				objects[0].ObjectGeometry[0].Triangle,
				objects[0].Mass);

			objects[0].SetBaseInertiaTensor(inertiaTensor1.GetInertiaTensor());
			objects[0].SetRotationMatrix(Quaternion.ConvertToMatrix(Quaternion.Normalize(objects[0].RotationStatus)));
			objects[0].SetInertiaTensor((objects[0].RotationMatrix * objects[0].BaseInertiaTensor) *
				Matrix3x3.Transpose(objects[0].RotationMatrix));

			//for (int j = 0; j < objects[0].ObjectGeometry[0].VertexPosition.Length; j++)
			//{
			//	Vector3 relPositionRotate = objects[0].RotationMatrix * objects[0].RelativePositions[j];
			//	objects[0].ObjectGeometry[0].SetVertexPosition(objects[0].Position + relPositionRotate, j);
			//}

			#endregion

			return objects;
		}

		private void getSimulationConstraints()
		{
		}

		private Geometry GetObjectGeometry(
			IShape shape,
            string fileName,
			float scale)
		{
			LoadResult objectGeometry = LoadObjSolid(fileName, scale);

			Vector3[] vertexStartPoint = new Vector3[objectGeometry.Vertices.Count];

			for (int i = 0; i < objectGeometry.Vertices.Count; i++)
			{
				vertexStartPoint[i] = new Vector3(
					objectGeometry.Vertices[i].X,
					objectGeometry.Vertices[i].Y,
					objectGeometry.Vertices[i].Z);
			}

			int[][] triangleIndex = new int[objectGeometry.Groups[0].Faces.Count][];

			for (int i = 0; i < objectGeometry.Groups[0].Faces.Count; i++)
			{
				triangleIndex[i] = new int[3];
				triangleIndex[i][0] = objectGeometry.Groups[0].Faces[i][0].VertexIndex - 1;
				triangleIndex[i][1] = objectGeometry.Groups[0].Faces[i][1].VertexIndex - 1;
				triangleIndex[i][2] = objectGeometry.Groups[0].Faces[i][2].VertexIndex - 1;
			}
            
			return new Geometry(
                shape,
				vertexStartPoint,
				triangleIndex,
                ObjectGeometryType.ConvexBody,
                false);
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

		#endregion
	}
}

