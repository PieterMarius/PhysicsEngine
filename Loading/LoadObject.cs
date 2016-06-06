using System;
using System.Xml;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using PhysicsEngineMathUtility;
using MonoPhysicsEngine;
using SimulationObjectDefinition;
using ObjLoader;
using ObjLoader.Loader.Loaders;
using Utility;

namespace Loading
{
	public class LoadObject
	{
		#region Private Fields

		private String nodePathObjects = "/RigidBodyEngine/ObjectsSettings/Object";
		private String nodePathJoints = "/RigidBodyEngine/JointsSettings/Joint";

		#region Object Attribute

		private String positionAttribute = "Position";
		private String linearVelAttribute = "LinearVelocity";
		private String angularVelAttribute = "AngularVelocity";
		private String rotationStatusAttribute = "RotationStatus";
		private String massAttribute = "Mass";
		private String restitutionCoeffAttribute = "RestitutionCoeff";
		private String dynamicFrictionAttribute = "DynamicFriction";
		private String staticFrictionAttribute = "StaticFriction";
		private String objectGeometryAttribute = "ObjectGeometry";
		private String scaleAttribute = "Scale";
		private String textureAttribute = "Texture";
		private String objectType = "ObjectType";
		private String excludeFromCollisionDetection = "ExludeFromCollisionDetection";

		#endregion

		#region Joint Attribute

		private String jointProperties = "JointProperties";
		private String objectIndexAAttribute = "ObjectIndexA";
		private String objectIndexBAttribute = "ObjectIndexB";
		private String jointType = "JointType";
		private String positionJointAttribute = "Position";
		private String actionAxis = "ActionAxis";
		private String restoreCoeffAttribute = "RestoreCoefficient";
		private String stretchCoeffAttribute = "StretchCoefficient";
		private String linearLimitMin = "LinearLimitMin";
		private String linearLimitMax = "LinearLimitMax";
		private String angularLimitMin = "AngularLimitMin";
		private String angularLimitMax = "AngularLimitMax";

		#endregion

		private Vector3[] translate;


		#endregion

		#region Public Fields

		public readonly String FileNameObjectProperties;

		#endregion

		#region Constructor

		public LoadObject (
			String fileNameObjectProperties)
		{
			this.FileNameObjectProperties = fileNameObjectProperties;
		}

		#endregion

		#region Public Methods

		public SimulationObject[] LoadSimulationObjects()
		{
			XmlDocument xmlDoc = new XmlDocument();
			xmlDoc.Load(this.FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			SimulationObject[] objects = new SimulationObject[xmlList.Count];

			this.translate = new Vector3[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++)
			{
				objects[i] = new SimulationObject();

				//Position
				objects [i].SetPosition (new Vector3 (
					Convert.ToDouble (xmlList [i] [this.positionAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [this.positionAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [this.positionAttribute].Attributes ["z"].Value)));

				//Linear Velocity
				objects [i].SetLinearVelocity (new Vector3 (
					Convert.ToDouble (xmlList [i] [this.linearVelAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [this.linearVelAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [this.linearVelAttribute].Attributes ["z"].Value)));

				//Angular Velocity
				objects [i].SetAngularVelocity (new Vector3 (
					Convert.ToDouble (xmlList [i] [this.angularVelAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [this.angularVelAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [this.angularVelAttribute].Attributes ["z"].Value)));

				//Rotation Status
				Vector3 versor = new Vector3 (
					Convert.ToDouble (xmlList [i] [this.rotationStatusAttribute].Attributes ["x"].Value),
					Convert.ToDouble (xmlList [i] [this.rotationStatusAttribute].Attributes ["y"].Value),
					Convert.ToDouble (xmlList [i] [this.rotationStatusAttribute].Attributes ["z"].Value));

				double angle = Convert.ToDouble(xmlList [i][this.rotationStatusAttribute].Attributes["angle"].Value);

				objects [i].SetRotationStatus (new Quaternion (versor, angle));

				//Object type
				objects [i].SetObjectType ((ObjectType)Convert.ToInt32 (xmlList [i] [this.objectType].InnerText));

				//Mass
				objects [i].SetMass (Convert.ToDouble (xmlList [i] [this.massAttribute].InnerText));

				//Restitution Coefficient
				objects [i].SetRestitutionCoeff (Convert.ToDouble (xmlList [i] [this.restitutionCoeffAttribute].InnerText));

				//Dynamic friction
				objects[i].SetDynamicFrictionCoeff (Convert.ToDouble(xmlList [i][this.dynamicFrictionAttribute].InnerText));

				//Static friction
				objects [i].SetStaticFrictionCoeff (Convert.ToDouble (xmlList [i] [this.staticFrictionAttribute].InnerText));

				//Collision detection
				objects[i].SetExcludeFromCollisionDetection (Convert.ToBoolean(xmlList [i] [this.excludeFromCollisionDetection].InnerText));

				//Scale
				float scale = Convert.ToSingle (xmlList [i] [this.scaleAttribute].InnerText);

				//Object geometry file name
				String geometryFileName = xmlList [i] [this.objectGeometryAttribute].InnerText;

				objects [i].ObjectGeometry = this.GetObjectGeometry (
					geometryFileName, 
					scale);

				//Inertia Tensor and Geometry

				InertiaTensor inertiaTensor = new InertiaTensor (
					objects [i].ObjectGeometry.VertexInitialPosition,
					objects [i].ObjectGeometry.Triangle,
					objects [i].Mass);

				//Traslo per normalizzare l'oggetto rispetto al suo centro di massa
				for (int j = 0; j < objects [i].ObjectGeometry.VertexInitialPosition.Length; j++) 
				{
					objects [i].ObjectGeometry.SetVertexInitialPosition (
						objects [i].ObjectGeometry.VertexInitialPosition [j] - inertiaTensor.GetMassCenter (),
						j);
				}

				InertiaTensor inertiaTensor1 = new InertiaTensor (
					objects [i].ObjectGeometry.VertexInitialPosition,
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

				AABB box = new AABB (objects [i].ObjectGeometry.VertexPosition.Min (point => point.x),
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

		public ObjectConstraint[] LoadSimulationJoints(
			SimulationObject[] objects)
		{
			XmlDocument xmlDoc = new XmlDocument();
			xmlDoc.Load(this.FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathJoints);

			ObjectConstraint[] joints = new ObjectConstraint[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++) 
			{
				//Object index A
				int indexA = Convert.ToInt32 (xmlList [i] [this.objectIndexAAttribute].InnerText);

				//Object index B
				int indexB = Convert.ToInt32 (xmlList [i] [this.objectIndexBAttribute].InnerText);

				XmlNodeList jointPropertiesList = xmlList [i].SelectNodes (this.jointProperties);

				IConstraint[] joint = new IConstraint[jointPropertiesList.Count];

				for (int j = 0; j < jointPropertiesList.Count; j++) {

					//Joint type
					JointType jointType = (JointType)Convert.ToInt32 (jointPropertiesList [j] [this.jointType].InnerText);

					//Restore coefficient
					double K = Convert.ToDouble (jointPropertiesList [j] [this.restoreCoeffAttribute].InnerText);

					//Stretch coefficient
					double C = Convert.ToDouble (jointPropertiesList [j] [this.stretchCoeffAttribute].InnerText);

					//Position
					Vector3 startAnchorPosition = new Vector3 (
						Convert.ToDouble (jointPropertiesList [j] [this.positionJointAttribute].Attributes ["x"].Value),
						Convert.ToDouble (jointPropertiesList [j] [this.positionJointAttribute].Attributes ["y"].Value),
						Convert.ToDouble (jointPropertiesList [j] [this.positionJointAttribute].Attributes ["z"].Value));

					//Action Axis
					Vector3 actionAxis = new Vector3 (
						Convert.ToDouble (jointPropertiesList [j] [this.actionAxis].Attributes ["x"].Value),
						Convert.ToDouble (jointPropertiesList [j] [this.actionAxis].Attributes ["y"].Value),
						Convert.ToDouble (jointPropertiesList [j] [this.actionAxis].Attributes ["z"].Value));

					switch (jointType) {
						case JointType.Fixed:
							joint [j] = new FixedJointConstraint (
								objects [indexA],
								objects [indexB],
								K,
								C);
							break;

						case JointType.BallAndSocket:
							joint [j] = new BallAndSocketConstraint (
								objects [indexA],
								objects [indexB],
								startAnchorPosition,
								K,
								C);
							break;

						case JointType.Slider:
							joint [j] = new SliderConstraint (
								objects [indexA],
								objects [indexB],
								startAnchorPosition,
								actionAxis,
								K,
								C,
								Convert.ToDouble (jointPropertiesList [j] [this.linearLimitMin].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.linearLimitMax].InnerText));
							break;

						case JointType.Piston:
							joint [j] = new PistonConstraint (
								objects [indexA],
								objects [indexB],
								startAnchorPosition,
								actionAxis,
								Convert.ToDouble (jointPropertiesList [j] [this.restoreCoeffAttribute].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.stretchCoeffAttribute].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.linearLimitMin].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.linearLimitMax].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMin].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMax].InnerText));						
							break;

						case JointType.Hinge:
							joint [j] = new HingeConstraint (
								objects [indexA],
								objects [indexB],
								startAnchorPosition,
								actionAxis,
								K,
								C,
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMin].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMax].InnerText),
								3.0,
								0.15);
							break;

						case JointType.Universal:
							joint [j] = new UniversalConstraint (
								objects [indexA],
								objects [indexB],
								startAnchorPosition,
								actionAxis,
								new Vector3 (1.0, 0.0, 0.0),
								K,
								C,
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMin].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMax].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMin].InnerText),
								Convert.ToDouble (jointPropertiesList [j] [this.angularLimitMax].InnerText));
							break;

						default:
							break;
					}
				}

				joints [i] = new ObjectConstraint (
					indexA,
					indexB,
					joint);
			}

			return joints;
		}

		public int[] GetOpenGLObjectList()
		{
			XmlDocument xmlDoc = new XmlDocument();
			xmlDoc.Load(this.FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			LoadResult[] loadObjects = new LoadResult[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++) 
			{
				//Object geometry file name
				String geometryFileName = xmlList [i] [this.objectGeometryAttribute].InnerText;

				//Scale
				float scale = Convert.ToSingle (xmlList [i] [this.scaleAttribute].InnerText);

				loadObjects[i]  = this.LoadObjSolid (geometryFileName, scale);
			}

			return OpenGLUtilities.LoadGLObjects (
				loadObjects,
				this.translate,
				xmlList.Count,
				true,
				false,
				true);
		}

		public int[] LoadTexture()
		{
			XmlDocument xmlDoc = new XmlDocument();
			xmlDoc.Load(this.FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			int[] textureID = new int[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++) 
			{
				//Object geometry file name
				String textureFileName = xmlList [i] [this.textureAttribute].InnerText;

				textureID[i] = OpenGLUtilities.LoadTexture(textureFileName);
			}

			return textureID;

		}

		#endregion

		#region Private Methods

		private LoadResult LoadObjSolid(
			String fileName,
			float scale)
		{
			ObjLoaderFactory objLoaderFactory = new ObjLoaderFactory ();
			var objLoader = objLoaderFactory.Create ();
			var fileStream = new FileStream (fileName, FileMode.OpenOrCreate);
			LoadResult solid = objLoader.Load (fileStream);
			fileStream.Close();

			OpenGLUtilities.UnitizeObject (ref solid);
			OpenGLUtilities.ScaleObject (ref solid, scale);

			return solid;
		}

		private ObjectGeometry GetObjectGeometry(
			String fileName,
			float scale)
		{
			LoadResult objectGeometry = this.LoadObjSolid (fileName, scale);

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
				vertexStartPoint,
				triangleIndex);
		}

		#endregion
	}
}

