using System;
using CollisionEngine;
using LCPSolver;
using MonoPhysicsEngine;
using SimulationObjectDefinition;

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

			return physicsEnvironment;
		}

		#endregion

		#region Private Methods

		private SimulationObject[] getSimulationObjects()
		{
			SimulationObject[] objects = new SimulationObject[5];

			//objects[i] = new SimulationObject();

			////Position
			//objects[i].SetPosition(new Vector3(
			//	Convert.ToDouble(xmlList[i][this.positionAttribute].Attributes["x"].Value),
			//	Convert.ToDouble(xmlList[i][this.positionAttribute].Attributes["y"].Value),
			//	Convert.ToDouble(xmlList[i][this.positionAttribute].Attributes["z"].Value)));

			////Linear Velocity
			//objects[i].SetLinearVelocity(new Vector3(
			//	Convert.ToDouble(xmlList[i][this.linearVelAttribute].Attributes["x"].Value),
			//	Convert.ToDouble(xmlList[i][this.linearVelAttribute].Attributes["y"].Value),
			//	Convert.ToDouble(xmlList[i][this.linearVelAttribute].Attributes["z"].Value)));

			////Angular Velocity
			//objects[i].SetAngularVelocity(new Vector3(
			//	Convert.ToDouble(xmlList[i][this.angularVelAttribute].Attributes["x"].Value),
			//	Convert.ToDouble(xmlList[i][this.angularVelAttribute].Attributes["y"].Value),
			//	Convert.ToDouble(xmlList[i][this.angularVelAttribute].Attributes["z"].Value)));

			////Rotation Status
			//Vector3 versor = new Vector3(
			//	Convert.ToDouble(xmlList[i][this.rotationStatusAttribute].Attributes["x"].Value),
			//	Convert.ToDouble(xmlList[i][this.rotationStatusAttribute].Attributes["y"].Value),
			//	Convert.ToDouble(xmlList[i][this.rotationStatusAttribute].Attributes["z"].Value));

			//double angle = Convert.ToDouble(xmlList[i][this.rotationStatusAttribute].Attributes["angle"].Value);

			//objects[i].SetRotationStatus(new Quaternion(versor, angle));

			////Object type
			//objects[i].SetObjectType((ObjectType)Convert.ToInt32(xmlList[i][this.objectType].InnerText));

			////Mass
			//objects[i].SetMass(Convert.ToDouble(xmlList[i][this.massAttribute].InnerText));

			////Restitution Coefficient
			//objects[i].SetRestitutionCoeff(Convert.ToDouble(xmlList[i][this.restitutionCoeffAttribute].InnerText));

			////Dynamic friction
			//objects[i].SetDynamicFrictionCoeff(Convert.ToDouble(xmlList[i][this.dynamicFrictionAttribute].InnerText));

			////Static friction
			//objects[i].SetStaticFrictionCoeff(Convert.ToDouble(xmlList[i][this.staticFrictionAttribute].InnerText));

			////Collision detection
			//objects[i].SetExcludeFromCollisionDetection(Convert.ToBoolean(xmlList[i][this.excludeFromCollisionDetection].InnerText));

			////Scale
			//float scale = Convert.ToSingle(xmlList[i][this.scaleAttribute].InnerText);

			////Object geometry file name
			//String geometryFileName = xmlList[i][this.objectGeometryAttribute].InnerText;

			//objects[i].ObjectGeometry = this.GetObjectGeometry(
			//	geometryFileName,
			//	scale);

			////Inertia Tensor and Geometry

			//InertiaTensor inertiaTensor = new InertiaTensor(
			//	objects[i].ObjectGeometry.VertexInitialPosition,
			//	objects[i].ObjectGeometry.Triangle,
			//	objects[i].Mass);

			////Traslo per normalizzare l'oggetto rispetto al suo centro di massa
			//for (int j = 0; j < objects[i].ObjectGeometry.VertexInitialPosition.Length; j++)
			//{
			//	objects[i].ObjectGeometry.SetVertexInitialPosition(
			//		objects[i].ObjectGeometry.VertexInitialPosition[j] - inertiaTensor.GetMassCenter(),
			//		j);
			//}

			//InertiaTensor inertiaTensor1 = new InertiaTensor(
			//	objects[i].ObjectGeometry.VertexInitialPosition,
			//	objects[i].ObjectGeometry.Triangle,
			//	objects[i].Mass);

			//objects[i].SetStartPosition(inertiaTensor1.GetMassCenter());
			//objects[i].SetBaseInertiaTensor(inertiaTensor1.GetInertiaTensor());

			//objects[i].SetRelativePosition();

			//objects[i].SetRotationMatrix(
			//	Quaternion.ConvertToMatrix(
			//		Quaternion.Normalize(objects[i].RotationStatus)));

			//objects[i].SetInertiaTensor(
			//	(objects[i].RotationMatrix * objects[i].BaseInertiaTensor) *
			//	Matrix3x3.Transpose(objects[i].RotationMatrix));

			//for (int j = 0; j < objects[i].ObjectGeometry.VertexPosition.Length; j++)
			//{
			//	Vector3 relPositionRotate = objects[i].RotationMatrix * objects[i].RelativePositions[j];
			//	objects[i].ObjectGeometry.SetVertexPosition(objects[i].Position + relPositionRotate, j);
			//}

			//AABB box = new AABB(objects[i].ObjectGeometry.VertexPosition.Min(point => point.x),
			//	objects[i].ObjectGeometry.VertexPosition.Max(point => point.x),
			//	objects[i].ObjectGeometry.VertexPosition.Min(point => point.y),
			//	objects[i].ObjectGeometry.VertexPosition.Max(point => point.y),
			//	objects[i].ObjectGeometry.VertexPosition.Min(point => point.z),
			//	objects[i].ObjectGeometry.VertexPosition.Max(point => point.z),
			//	false);

			//objects[i].ObjectGeometry.SetAABB(box);

			return objects;
		}

		private void getSimulationConstraints()
		{
		}

		#endregion
	}
}

