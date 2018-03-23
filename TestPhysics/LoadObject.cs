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

using System;
using System.Xml;
using System.Linq;
using SharpEngineMathUtility;
using SharpPhysicsEngine;
using SharpPhysicsEngine.ShapeDefinition;
using Utility;
using SharpPhysicsEngine.Wrapper;
using SharpPhysicsEngine.Wrapper.Joint;

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

		public ICollisionShape[] LoadSimulationObjects()
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

            ICollisionShape[] objects = new ICollisionShape[xmlList.Count];
						
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

				GeometryProperties[] objGeometry = new GeometryProperties[xmlGeometryList.Count];
				double[] mass = new double[xmlGeometryList.Count];
				Vector3[] startCompositePosition = new Vector3[xmlGeometryList.Count];
                
				if (xmlGeometryList.Count > 1)
					objects[i] = new CompoundRigidCollisionShape();
				else
                {
                    if ((ObjectType)Convert.ToInt32(xmlList[i][objectType].InnerText) == ObjectType.RigidBody)
                        objects[i] = new RigidCollisionShape();
                    else if ((ObjectType)Convert.ToInt32(xmlList[i][objectType].InnerText) == ObjectType.StaticBody)
                        objects[i] = new StaticCollisionShape();
                }
					
				
				for (int j = 0; j < xmlGeometryList.Count; j++)
				{
					//Scale
					float scale = Convert.ToSingle(xmlGeometryList[j][scaleAttribute].InnerText);

					//Object geometry file name
					string geometryFileName = xmlGeometryList[j][objectGeometryAttribute].InnerText;

					//Object mass
					mass[j] = Convert.ToDouble(xmlGeometryList[j][massAttribute].InnerText);

					objGeometry[j] = GetObjectGeometry(geometryFileName, scale);

					startCompositePosition[j] = new Vector3(
						Convert.ToDouble(xmlGeometryList[j][compositePosition].Attributes["x"].Value),
						Convert.ToDouble(xmlGeometryList[j][compositePosition].Attributes["y"].Value),
						Convert.ToDouble(xmlGeometryList[j][compositePosition].Attributes["z"].Value));
				}

				objects[i].SetPosition(position);
				objects[i].SetRotationStatus(new Quaternion(versor, angle));

				if (xmlGeometryList.Count > 1)
				{
					((CompoundRigidCollisionShape)objects[i]).SetPartialMass(mass);
					((CompoundRigidCollisionShape)objects[i]).SetCompoundPosition(startCompositePosition);
                    ((CompoundRigidCollisionShape)objects[i]).SetGeometry(
                                    objGeometry.Select(x => x.VertexPoint).ToList(),
                                    objGeometry.Select(x => x.TriagleIdx).ToList());
				}
				else
				{
					objects[i].SetMass(mass[0]);
                    objects[i].SetGeometry(objGeometry[0].VertexPoint, objGeometry[0].TriagleIdx);
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
				objects[i].ExcludeFromCollisionDetection (Convert.ToBoolean(xmlList [i] [excludeFromCollisionDetection].InnerText));

				//Baumgarte Stabilization value
				objects[i].SetErrorReductionParam(0.5);

			}

			return objects;
		}

		public ICollisionJoint[] LoadSimulationJoints(
			ICollisionShape[] objects)
		{
			var xmlDoc = new XmlDocument();
			xmlDoc.Load(FileNameObjectProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathJoints);

            ICollisionJoint[] joints = new ICollisionJoint[xmlList.Count];

			for (int i = 0; i < xmlList.Count; i++)
			{
				//Object index A
				int indexA = Convert.ToInt32(xmlList[i][objectIndexAAttribute].InnerText);

				//Object index B
				int indexB = Convert.ToInt32(xmlList[i][objectIndexBAttribute].InnerText);

				XmlNodeList jointPropertiesList = xmlList[i].SelectNodes(jointProperties);

                ICollisionJoint[] joint = new ICollisionJoint[jointPropertiesList.Count];

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
							joint [j] = new FixedJoint (
								objects[indexA],
								objects[indexB],
								K,
								C);
							break;

						case JointType.BallAndSocket:
                            joint[j] = new BallAndSocketJoint(
                                objects[indexA],
								objects[indexB],
								startAnchorPosition,
								K,
								C);
                            break;

						case JointType.Slider:
                            joint[j] = new SliderJoint(
                                objects[indexA],
                                objects[indexB],
                                startAnchorPosition,
                                actionAxis,
                                K,
                                C);

                            joint[j].SetLinearLimit(Convert.ToDouble(jointPropertiesList[j][linearLimitMin].InnerText), Convert.ToDouble(jointPropertiesList[j][linearLimitMax].InnerText));

							break;

						case JointType.Piston:
                            joint[j] = new PistonJoint(
                                objects[indexA],
                                objects[indexB],
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
                            joint[j] = new HingeJoint(
                                objects[indexA],
                                objects[indexB],
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
                            joint[j] = new UniversalJoint(
                                objects[indexA],
                                objects[indexB],
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
                            joint[j] = new Hinge2Joint(
                                objects[indexA],
                                objects[indexB],
                                startAnchorPosition,
                                actionAxis,
                                new Vector3(1.0, 0.0, 0.0),
                                K,
                                1.0,
                                C);

                            joint[j].SetAxis1AngularLimit(
								Convert.ToDouble(jointPropertiesList[j][angularLimitMin].InnerText),
								Convert.ToDouble(jointPropertiesList[j][angularLimitMax].InnerText));

							//joint[j].SetAxis2Motor(4.0, 3.0);

							break;

						case JointType.Angular:
                            joint[j] = new AngularJoint(
                                objects[indexA],
                                objects[indexB],
                                startAnchorPosition,
                                new Vector3(1.0, 0.0, 0.0),
                                new Vector3(0.0, 1.0, 0.0),
                                0.16,
                                0.008,
                                0.008);

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

        	#endregion
		}
}

