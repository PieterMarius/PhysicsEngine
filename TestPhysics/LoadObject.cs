using System;
using System.Xml;
using PhysicsEngineMathUtility;
using SharpPhysicsEngine;
using ShapeDefinition;
using Utility;

namespace TestPhysics
{
	public class LoadObject
	{
		#region Private Fields

		string nodePathObjects = "/RigidBodyEngine/ObjectsSettings/Object";
        string nodePathGeometry = "ObjectGeometry";
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
        string compositePosition = "CompositePosition";

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

		public IShape[] LoadSimulationObjects()
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			IShape[] objects = new IShape[xmlList.Count];
            			
			for (int i = 0; i < xmlList.Count; i++)
			{
                //Rotation Status
                var versor = new Vector3(
					Convert.ToDouble(xmlList[i][rotationStatusAttribute].Attributes["x"].Value),
					Convert.ToDouble(xmlList[i][rotationStatusAttribute].Attributes["y"].Value),
					Convert.ToDouble(xmlList[i][rotationStatusAttribute].Attributes["z"].Value));

				double angle = Convert.ToDouble(xmlList[i][rotationStatusAttribute].Attributes["angle"].Value);

				//Position
				var position = new Vector3(
					Convert.ToDouble(xmlList[i][positionAttribute].Attributes["x"].Value),
					Convert.ToDouble(xmlList[i][positionAttribute].Attributes["y"].Value),
					Convert.ToDouble(xmlList[i][positionAttribute].Attributes["z"].Value));

                XmlNodeList xmlGeometryList = xmlList[i].SelectNodes(nodePathGeometry);

                IGeometry[] objGeometry = new Geometry[xmlGeometryList.Count];
                double[] mass = new double[xmlGeometryList.Count];
                Vector3[] startCompositePosition = new Vector3[xmlGeometryList.Count];

                if (xmlGeometryList.Count > 1)
                    objects[i] = new CompoundShape((ObjectType)Convert.ToInt32(xmlList[i][objectType].InnerText));
                else
                    objects[i] = new ConvexShape((ObjectType)Convert.ToInt32(xmlList[i][objectType].InnerText));
                
                for (int j = 0; j < xmlGeometryList.Count; j++)
                {
                    //Scale
                    float scale = Convert.ToSingle(xmlGeometryList[j][scaleAttribute].InnerText);

                    //Object geometry file name
                    string geometryFileName = xmlGeometryList[j][objectGeometryAttribute].InnerText;

                    //Object mass
                    mass[j] = Convert.ToDouble(xmlGeometryList[j][massAttribute].InnerText);

                    objGeometry[j] = GetObjectGeometry(objects[i], geometryFileName, scale, ObjectGeometryType.ConvexBody);

                    startCompositePosition[j] = new Vector3(
                        Convert.ToDouble(xmlGeometryList[j][compositePosition].Attributes["x"].Value),
                        Convert.ToDouble(xmlGeometryList[j][compositePosition].Attributes["y"].Value),
                        Convert.ToDouble(xmlGeometryList[j][compositePosition].Attributes["z"].Value));
                }

                objects[i].SetPosition(position);
                objects[i].SetRotationStatus(new Quaternion(versor, angle));

                if (xmlGeometryList.Count > 1)
                {
                    ((ICompoundShape)objects[i]).SetPartialMass(mass);
                    ((ICompoundShape)objects[i]).SetCompoundPosition(startCompositePosition);
                    ((ICompoundShape)objects[i]).SetObjectGeometry(objGeometry);
                }
                else
                {
                    ((IConvexShape)objects[i]).SetMass(mass[0]);
                    ((IConvexShape)objects[i]).SetObjectGeometry(objGeometry[0]);
                }
                                
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

				//Restitution Coefficient
				objects [i].SetRestitutionCoeff (Convert.ToDouble (xmlList [i] [restitutionCoeffAttribute].InnerText));

				//Dynamic friction
				objects[i].SetDynamicFrictionCoeff (Convert.ToDouble(xmlList [i][dynamicFrictionAttribute].InnerText));

				//Static friction
				objects [i].SetStaticFrictionCoeff (Convert.ToDouble (xmlList [i] [staticFrictionAttribute].InnerText));

				//Collision detection
				objects[i].SetExcludeFromCollisionDetection (Convert.ToBoolean(xmlList [i] [excludeFromCollisionDetection].InnerText));

				//Baumgarte Stabilization value
				objects[i].SetRestoreCoeff(30.0);

            }

			return objects;
		}

		public IConstraint[] LoadSimulationJoints(
			IShape[] objects)
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

                        case JointType.Angular:
                            joint[j] = new AngularConstraint(
                                indexA, 
                                indexB, 
                                objects, 
                                startAnchorPosition,
                                new Vector3(1.0, 0.0, 0.0), 
                                new Vector3(0.0, 1.0, 0.0), 
                                10.0,
                                0.5,
                                0.5);

                            break;

					}
					joints[i] = joint[j];
				}


			}

			return joints;
		}

        public int[][] LoadTexture()
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			int[][] textureID = new int[xmlList.Count][];

			for (int i = 0; i < xmlList.Count; i++) 
			{
                XmlNodeList xmlGeometryList = xmlList[i].SelectNodes(nodePathGeometry);

                textureID[i] = new int[xmlGeometryList.Count];

                for (int j = 0; j < xmlGeometryList.Count; j++)
                {
                    //Object geometry file name
                    string textureFileName = xmlGeometryList[j][textureAttribute].InnerText;

                    textureID[i][j] = OpenGLUtilities.LoadTexture(textureFileName);
                }
                   
			}

			return textureID;

		}

        public int[][] GetOpenGLObjectList()
        {
            var xmlDoc = new XmlDocument();
            xmlDoc.Load(FileNameObjectProperties);

            XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

            ObjImporter.meshStruct[][] loadObjects = new ObjImporter.meshStruct[xmlList.Count][];

            Vector3[][] translate = new Vector3[xmlList.Count][];

            for (int i = 0; i < xmlList.Count; i++)
            {
                XmlNodeList xmlGeometryList = xmlList[i].SelectNodes(nodePathGeometry);

                translate[i] = new Vector3[xmlGeometryList.Count];
                loadObjects[i] = new ObjImporter.meshStruct[xmlGeometryList.Count];

                for (int j = 0; j < xmlGeometryList.Count; j++)
                {
                    //Object geometry file name
                    string geometryFileName = xmlGeometryList[j][objectGeometryAttribute].InnerText;

                    //Scale
                    float scale = Convert.ToSingle(xmlGeometryList[j][scaleAttribute].InnerText);

                    loadObjects[i][j] = LoadObjSolid(geometryFileName, scale);
                }
            }

            return OpenGLUtilities.LoadGLObjects(
                loadObjects,
                translate,
                xmlList.Count,
                true,
                false,
                true);
        }

        public static int[][] GetOpenGLObjectList(
            string fileName, 
            double scale)
        {
            ObjImporter.meshStruct[][] loadObjects = new ObjImporter.meshStruct[1][];

            loadObjects[0] = new ObjImporter.meshStruct[1];

            loadObjects[0][0] = LoadObjSolid(fileName, scale);

            Vector3[][] translate = new Vector3[1][];

            translate[0] = new Vector3[] { new Vector3(0, 0, 0) };

            return OpenGLUtilities.LoadGLObjects(
                loadObjects,
                translate,
                1,
                true,
                false,
                true);
        }

        #endregion

        #region Private Methods

        private static ObjImporter.meshStruct LoadObjSolid(
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

        public static IGeometry GetObjectGeometry(
            IShape shape,
            string fileName,
            float scale,
            ObjectGeometryType geometryType)
        {
            ObjImporter importer = new ObjImporter();
            ObjImporter.meshStruct mesh = importer.ImportFile(fileName);

            Vector3[] vertexStartPoint = new Vector3[mesh.vertices.Length];

            for (int i = 0; i < mesh.vertices.Length; i++)
                vertexStartPoint[i] = mesh.vertices[i];

            OpenGLUtilities.UnitizeObject(ref vertexStartPoint);
            OpenGLUtilities.ScaleObject(ref vertexStartPoint, scale);
            
            int nTriangle = mesh.faceData.Length / 3;
            int[][] triangleIndex = new int[nTriangle][];

            for (int i = 0; i < nTriangle; i++)
            {
                triangleIndex[i] = new int[3];
                triangleIndex[i][0] = (int)mesh.faceData[i * 3].x - 1;
                triangleIndex[i][1] = (int)mesh.faceData[(i * 3) + 1].x - 1;
                triangleIndex[i][2] = (int)mesh.faceData[(i * 3) + 2].x - 1;
            }

            return new Geometry(
                shape,
                vertexStartPoint,
                triangleIndex,
                geometryType,
                true);
        }

            #endregion
        }
}

