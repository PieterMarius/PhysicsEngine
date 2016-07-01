using System;
using System.Xml;
using System.IO;
using System.Linq;
using PhysicsEngineMathUtility;
using MonoPhysicsEngine;
using SimulationObjectDefinition;
using ObjLoader.Loader.Loaders;
using Utility;

namespace TestPhysics
{
	public class LoadObject
	{
		#region Private Fields

		string nodePathObjects = "/RigidBodyEngine/ObjectsSettings/Object";
		string nodePathJoints = "/RigidBodyEngine/JointsSettings/Joint";

		#region Object Attribute

		string positionAttribute = "Position";
		string linearVelAttribute = "LinearVelocity";
		string angularVelAttribute = "AngularVelocity";
		string rotationStatusAttribute = "RotationStatus";
		string massAttribute = "Mass";
		string restitutionCoeffAttribute = "RestitutionCoeff";
		string dynamicFrictionAttribute = "DynamicFriction";
		string staticFrictionAttribute = "StaticFriction";
		string objectGeometryAttribute = "ObjectGeometry";
		string scaleAttribute = "Scale";
		string textureAttribute = "Texture";
		string objectType = "ObjectType";
		string excludeFromCollisionDetection = "ExludeFromCollisionDetection";

		#endregion

		#region Joint Attribute

		string jointProperties = "JointProperties";
		string objectIndexAAttribute = "ObjectIndexA";
		string objectIndexBAttribute = "ObjectIndexB";
		string jointType = "JointType";
		string positionJointAttribute = "Position";
		string actionAxis = "ActionAxis";
		string restoreCoeffAttribute = "RestoreCoefficient";
		string stretchCoeffAttribute = "StretchCoefficient";
		string linearLimitMin = "LinearLimitMin";
		string linearLimitMax = "LinearLimitMax";
		string angularLimitMin = "AngularLimitMin";
		string angularLimitMax = "AngularLimitMax";

		#endregion

		Vector3[] translate;


		#endregion

		#region Public Fields

		public readonly string FileNameObjectProperties;

		#endregion

		#region Constructor

		public LoadObject (
			string fileNameObjectProperties)
		{
			FileNameObjectProperties = fileNameObjectProperties;
		}

		#endregion

		#region Public Methods

		public SimulationObject[] LoadSimulationObjects()
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			SimulationObject[] objects = new SimulationObject[xmlList.Count];

			translate = new Vector3[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++)
			{
				objects[i] = new SimulationObject();

				//Position
				objects [i].SetPosition (new Vector3 (
					Convert.ToDouble (xmlList [i] [positionAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [positionAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [positionAttribute].Attributes ["z"].Value)));

				//Linear Velocity
				objects [i].SetLinearVelocity (new Vector3 (
					Convert.ToDouble (xmlList [i] [linearVelAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [linearVelAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [linearVelAttribute].Attributes ["z"].Value)));

				//Angular Velocity
				objects [i].SetAngularVelocity (new Vector3 (
					Convert.ToDouble (xmlList [i] [angularVelAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [angularVelAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [angularVelAttribute].Attributes ["z"].Value)));

				//Rotation Status
				var versor = new Vector3 (
					Convert.ToDouble (xmlList [i] [rotationStatusAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [rotationStatusAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [rotationStatusAttribute].Attributes ["z"].Value));

				double angle = Convert.ToDouble(xmlList [i][rotationStatusAttribute].Attributes["angle"].Value);

				objects [i].SetRotationStatus (new Quaternion (versor, angle));

				//Object type
				objects [i].SetObjectType ((ObjectType)Convert.ToInt32 (xmlList [i] [objectType].InnerText));

				//Mass
				objects [i].SetMass (Convert.ToDouble (xmlList [i] [massAttribute].InnerText));

				//Restitution Coefficient
				objects [i].SetRestitutionCoeff (Convert.ToDouble (xmlList [i] [restitutionCoeffAttribute].InnerText));

				//Dynamic friction
				objects[i].SetDynamicFrictionCoeff (Convert.ToDouble(xmlList [i][dynamicFrictionAttribute].InnerText));

				//Static friction
				objects [i].SetStaticFrictionCoeff (Convert.ToDouble (xmlList [i] [staticFrictionAttribute].InnerText));

				//Collision detection
				objects[i].SetExcludeFromCollisionDetection (Convert.ToBoolean(xmlList [i] [excludeFromCollisionDetection].InnerText));

				//Scale
				float scale = Convert.ToSingle (xmlList [i] [scaleAttribute].InnerText);

				//Object geometry file name
				string geometryFileName = xmlList [i] [objectGeometryAttribute].InnerText;

				objects [i].ObjectGeometry = GetObjectGeometry (
					geometryFileName, 
					scale);

				//Inertia Tensor and Geometry

				var inertiaTensor = new InertiaTensor (
					objects [i].ObjectGeometry.VertexPosition,
					objects [i].ObjectGeometry.Triangle,
					objects [i].Mass);

				//Traslo per normalizzare l'oggetto rispetto al suo centro di massa
				for (int j = 0; j < objects [i].ObjectGeometry.VertexPosition.Length; j++) 
				{
					objects [i].ObjectGeometry.SetVertexPosition (
						objects [i].ObjectGeometry.VertexPosition [j] - inertiaTensor.GetMassCenter (),
						j);
				}

				var inertiaTensor1 = new InertiaTensor (
					objects [i].ObjectGeometry.VertexPosition,
					objects [i].ObjectGeometry.Triangle,
					objects [i].Mass);

				objects [i].SetStartPosition (inertiaTensor1.GetMassCenter ());
				objects [i].SetBaseInertiaTensor (inertiaTensor1.GetInertiaTensor ());

				objects [i].SetRelativePosition ();

				objects [i].SetRotationMatrix (
					Quaternion.ConvertToMatrix (
						Quaternion.Normalize (objects [i].RotationStatus)));

				objects [i].SetInertiaTensor (
					(objects [i].RotationMatrix * objects [i].BaseInertiaTensor) *
					Matrix3x3.Transpose (objects [i].RotationMatrix));

				for (int j = 0; j < objects [i].ObjectGeometry.VertexPosition.Length; j++) 
				{
					Vector3 relPositionRotate = objects [i].RotationMatrix * objects [i].RelativePositions [j];
					objects [i].ObjectGeometry.SetVertexPosition (objects [i].Position + relPositionRotate, j);
				}

				var box = new AABB (objects [i].ObjectGeometry.VertexPosition.Min (point => point.x),
					objects [i].ObjectGeometry.VertexPosition.Max (point => point.x),
					objects [i].ObjectGeometry.VertexPosition.Min (point => point.y),
					objects [i].ObjectGeometry.VertexPosition.Max (point => point.y),
					objects [i].ObjectGeometry.VertexPosition.Min (point => point.z),
					objects [i].ObjectGeometry.VertexPosition.Max (point => point.z),
					false);

				objects [i].ObjectGeometry.SetAABB (box);

			}

			return objects;
		}

		public IConstraint[] LoadSimulationJoints(
			SimulationObject[] objects)
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathJoints);

			IConstraint[] joints = new IConstraint[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++)
			{
				//Object index A
				int indexA = Convert.ToInt32(xmlList[i][objectIndexAAttribute].InnerText);

				//Object index B
				int indexB = Convert.ToInt32(xmlList[i][objectIndexBAttribute].InnerText);

				XmlNodeList jointPropertiesList = xmlList[i].SelectNodes(jointProperties);

				IConstraint[] joint = new IConstraint[jointPropertiesList.Count];

				for (int j = 0; j < jointPropertiesList.Count; j++)
				{

					//Joint type
					var jointType = (JointType)Convert.ToInt32(jointPropertiesList[j][this.jointType].InnerText);

					//Restore coefficient
					double K = Convert.ToDouble(jointPropertiesList[j][restoreCoeffAttribute].InnerText);

					//Stretch coefficient
					double C = Convert.ToDouble(jointPropertiesList[j][stretchCoeffAttribute].InnerText);

					//Position
					var startAnchorPosition = new Vector3(
						Convert.ToDouble(jointPropertiesList[j][positionJointAttribute].Attributes["x"].Value),
						Convert.ToDouble(jointPropertiesList[j][positionJointAttribute].Attributes["y"].Value),
						Convert.ToDouble(jointPropertiesList[j][positionJointAttribute].Attributes["z"].Value));

					//Action Axis
					var actionAxis = new Vector3(
						Convert.ToDouble(jointPropertiesList[j][this.actionAxis].Attributes["x"].Value),
						Convert.ToDouble(jointPropertiesList[j][this.actionAxis].Attributes["y"].Value),
						Convert.ToDouble(jointPropertiesList[j][this.actionAxis].Attributes["z"].Value));

					switch (jointType) {
						case JointType.Fixed:
							joint [j] = new FixedJointConstraint (
								indexA,
								indexB,
								objects,
								K,
								C);
							break;

						case JointType.BallAndSocket:
							joint [j] = new BallAndSocketConstraint (
								indexA,
								indexB,
								objects,
								startAnchorPosition,
								K,
								C);
							break;

						case JointType.Slider:
							joint [j] = new SliderConstraint (
								indexA,
								indexB,
								objects,
								startAnchorPosition,
								actionAxis,
								K,
								C);

							joint[j].SetLinearLimit(Convert.ToDouble(jointPropertiesList[j][linearLimitMin].InnerText), Convert.ToDouble(jointPropertiesList[j][linearLimitMax].InnerText));

							break;

						case JointType.Piston:
							joint[j] = new PistonConstraint(
								indexA,
								indexB,
								objects,
								startAnchorPosition,
								actionAxis,
								K,
								C);

							joint[j].SetAxis1AngularLimit(
								Convert.ToDouble(jointPropertiesList[j][angularLimitMin].InnerText),
								Convert.ToDouble(jointPropertiesList[j][angularLimitMax].InnerText));

							joint[j].SetLinearLimit(
								Convert.ToDouble(jointPropertiesList[j][linearLimitMin].InnerText),
								Convert.ToDouble(jointPropertiesList[j][linearLimitMax].InnerText));
							
							break;

						case JointType.Hinge:
							joint[j] = new HingeConstraint(
								indexA,
								indexB,
								objects,
								startAnchorPosition,
								actionAxis,
								K,
								C);

							joint[j].SetAxis1AngularLimit(
								Convert.ToDouble(jointPropertiesList[j][angularLimitMin].InnerText),
								Convert.ToDouble(jointPropertiesList[j][angularLimitMax].InnerText));

							joint[j].SetAxis1Motor(3.0, 0.15);
							break;

						case JointType.Universal:
							joint[j] = new UniversalConstraint(
								indexA,
								indexB,
								objects,
								startAnchorPosition,
								actionAxis,
								new Vector3(1.0, 0.0, 0.0),
								K,
								C);

							joint[j].SetAxis1AngularLimit(
								Convert.ToDouble(jointPropertiesList[j][angularLimitMin].InnerText),
								Convert.ToDouble(jointPropertiesList[j][angularLimitMax].InnerText));
							joint[j].SetAxis2AngularLimit(
								Convert.ToDouble(jointPropertiesList[j][angularLimitMin].InnerText),
								Convert.ToDouble(jointPropertiesList[j][angularLimitMax].InnerText));
							break;

						case JointType.Hinge2:
							joint [j] = new Hinge2Constraint (
								indexA,
								indexB,
								objects,
								startAnchorPosition,
								actionAxis,
								new Vector3 (1.0, 0.0, 0.0),
								K,
								1.0,
								C);

							joint[j].SetAxis1AngularLimit(
								Convert.ToDouble(jointPropertiesList[j][angularLimitMin].InnerText),
								Convert.ToDouble(jointPropertiesList[j][angularLimitMax].InnerText));

							//joint[j].SetAxis2Motor(4.0, 3.0);

							break;

					}
					joints[i] = joint[j];
				}


			}

			return joints;
		}

		public int[] GetOpenGLObjectList()
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			LoadResult[] loadObjects = new LoadResult[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++) 
			{
				//Object geometry file name
				string geometryFileName = xmlList [i] [objectGeometryAttribute].InnerText;

				//Scale
				float scale = Convert.ToSingle (xmlList [i] [scaleAttribute].InnerText);

				loadObjects[i]  = LoadObjSolid (geometryFileName, scale);
			}

			return OpenGLUtilities.LoadGLObjects (
				loadObjects,
				translate,
				xmlList.Count,
				true,
				false,
				true);
		}

		public int[] LoadTexture()
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			int[] textureID = new int[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++) 
			{
				//Object geometry file name
				string textureFileName = xmlList [i] [textureAttribute].InnerText;

				textureID[i] = OpenGLUtilities.LoadTexture(textureFileName);
			}

			return textureID;

		}

		#endregion

		#region Private Methods

		private LoadResult LoadObjSolid(
			string fileName,
			float scale)
		{
			var objLoaderFactory = new ObjLoaderFactory ();
			var objLoader = objLoaderFactory.Create ();
			var fileStream = new FileStream (fileName, FileMode.OpenOrCreate);
			LoadResult solid = objLoader.Load (fileStream);
			fileStream.Close();

			OpenGLUtilities.UnitizeObject (ref solid);
			OpenGLUtilities.ScaleObject (ref solid, scale);

			return solid;
		}

		private ObjectGeometry GetObjectGeometry(
			string fileName,
			float scale)
		{
			LoadResult objectGeometry = LoadObjSolid (fileName, scale);

			Vector3[] vertexStartPoint = new Vector3[objectGeometry.Vertices.Count];

			for (int i = 0; i < objectGeometry.Vertices.Count; i++) 
			{
				vertexStartPoint [i] = new Vector3 (
					objectGeometry.Vertices [i].X,
					objectGeometry.Vertices [i].Y,
					objectGeometry.Vertices [i].Z);
			}

			int[][] triangleIndex = new int[objectGeometry.Groups [0].Faces.Count][];

			for (int i = 0; i < objectGeometry.Groups [0].Faces.Count; i++) 
			{
				triangleIndex [i] = new int[3];
				triangleIndex [i] [0] = objectGeometry.Groups [0].Faces [i] [0].VertexIndex - 1;
				triangleIndex [i] [1] = objectGeometry.Groups [0].Faces [i] [1].VertexIndex - 1;
				triangleIndex [i] [2] = objectGeometry.Groups [0].Faces [i] [2].VertexIndex - 1;
			}

			return new ObjectGeometry (
				vertexStartPoint,
				triangleIndex);
		}

		#endregion
	}
}

