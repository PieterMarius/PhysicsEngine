using System.IO;
using System.Linq;
using CollisionEngine;
using LCPSolver;
using MonoPhysicsEngine;
using ObjLoader.Loader.Loaders;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;
using Utility;

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

		public PhysicsEngine GetPhysicsEnvironment()
		{
			var simulationParam = new SimulationParameters();
			var solverParameters = new SolverParameters();
			var collisionEngineParam = new CollisionEngineParameters();

			var physicsEnvironment = new PhysicsEngine(
													simulationParam,
													collisionEngineParam,
													solverParameters);

			physicsEnvironment.AddObject(getSimulationObjects()[0]);
			physicsEnvironment.RemoveObject(0);

			return physicsEnvironment;
		}

		#endregion

		#region Private Methods

		private SimulationObject[] getSimulationObjects()
		{
			SimulationObject[] objects = new SimulationObject[1];

			#region Terrain Base

			objects[0] = new SimulationObject(
				ObjectType.StaticRigidBody,
				geometry: GetObjectGeometry("cube.obj", 25),
				mass: 1000.0,
				position: new Vector3(0.0, -4.0, 0.0),
				rotationStatus: new Quaternion(new Vector3(0.0, 0.0, 0.0), 0.0));
			
			objects[0].SetLinearVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[0].SetAngularVelocity(new Vector3(0.0, 0.0, 0.0));
			objects[0].SetRestitutionCoeff(0.1);
			objects[0].SetDynamicFrictionCoeff(1.0);
			objects[0].SetStaticFrictionCoeff(1.0);
			objects[0].SetExcludeFromCollisionDetection(false);

			var inertiaTensor = new InertiaTensor(
				objects[0].ObjectGeometry.VertexPosition,
				objects[0].ObjectGeometry.Triangle,
				objects[0].Mass);

			//Traslo per normalizzare l'oggetto rispetto al suo centro di massa
			for (int j = 0; j < objects[0].ObjectGeometry.VertexPosition.Length; j++)
			{
				objects[0].ObjectGeometry.SetVertexPosition(
					objects[0].ObjectGeometry.VertexPosition[j] - inertiaTensor.GetMassCenter(),
					j);
			}

			var inertiaTensor1 = new InertiaTensor(
				objects[0].ObjectGeometry.VertexPosition,
				objects[0].ObjectGeometry.Triangle,
				objects[0].Mass);

			objects[0].SetStartPosition(inertiaTensor1.GetMassCenter());
			objects[0].SetBaseInertiaTensor(inertiaTensor1.GetInertiaTensor());
			objects[0].SetRotationMatrix(Quaternion.ConvertToMatrix(Quaternion.Normalize(objects[0].RotationStatus)));
			objects[0].SetInertiaTensor((objects[0].RotationMatrix * objects[0].BaseInertiaTensor) *
				Matrix3x3.Transpose(objects[0].RotationMatrix));

			for (int j = 0; j < objects[0].ObjectGeometry.VertexPosition.Length; j++)
			{
				Vector3 relPositionRotate = objects[0].RotationMatrix * objects[0].RelativePositions[j];
				objects[0].ObjectGeometry.SetVertexPosition(objects[0].Position + relPositionRotate, j);
			}

			var box = new AABB(objects[0].ObjectGeometry.VertexPosition.Min(point => point.x),
				objects[0].ObjectGeometry.VertexPosition.Max(point => point.x),
				objects[0].ObjectGeometry.VertexPosition.Min(point => point.y),
				objects[0].ObjectGeometry.VertexPosition.Max(point => point.y),
				objects[0].ObjectGeometry.VertexPosition.Min(point => point.z),
				objects[0].ObjectGeometry.VertexPosition.Max(point => point.z),
				false);

			objects[0].ObjectGeometry.SetAABB(box);

			#endregion

			return objects;
		}

		private void getSimulationConstraints()
		{
		}

		private ObjectGeometry GetObjectGeometry(
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

			return new ObjectGeometry(
				vertexStartPoint,
				triangleIndex);
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

