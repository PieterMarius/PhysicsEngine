using System;
using System.Xml;
using MonoPhysicsEngine;
using PhysicsEngineMathUtility;

namespace TestPhysics
{
	public class LoadEngineConfig
	{
		#region Private Fields

		private String nodePathObjects = "/RigidBodyEngine/Parameters";

		#endregion

		#region Engine Fields

		private String timeStep = "TimeStep";
		private String collisionDistance = "CollisionDistance";
		private String CFM = "CFM";
		private String baumStabilization = "BaumStabilization";
		private String linearVelocityStabilization = "LinearVelociyStabilize";
		private String angularVelocityStabilization = "AngularVelociyStabilize";
		private String shiftToStaticFrictionTolerance = "ShiftToStaticFrictionTolerance";
		private String externalForce = "ExternalForce";
		private String discreteCCD = "DiscreteCCD";
		private String maxThreadNumber = "MaxThreadNumber";

		#endregion

		#region Public Fields

		public readonly String FileNameEngineProperties;

		#endregion

		#region Constructor

		public LoadEngineConfig (
			String fileNameEngineProperties)
		{
			this.FileNameEngineProperties = fileNameEngineProperties;
		}

		#endregion

		#region Public Methods

		public SimulationParameters ReadEngineConfig()
		{
			XmlDocument xmlDoc = new XmlDocument();
			xmlDoc.Load(this.FileNameEngineProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			SimulationParameters simulationParameters = new SimulationParameters (
                                Convert.ToDouble (xmlList [0] [this.timeStep].InnerText),
                                Convert.ToDouble (xmlList [0] [this.CFM].InnerText),
                                Convert.ToDouble (xmlList [0] [this.baumStabilization].InnerText),
                                Convert.ToDouble (xmlList [0] [this.linearVelocityStabilization].InnerText),
                                Convert.ToDouble (xmlList [0] [this.angularVelocityStabilization].InnerText),
                                Convert.ToDouble (xmlList [0] [this.shiftToStaticFrictionTolerance].InnerText),
                                Convert.ToBoolean (xmlList [0] [this.discreteCCD].InnerText),
                                Convert.ToDouble (xmlList [0] [this.collisionDistance].InnerText),
								//TODO
								Convert.ToDouble(xmlList[0][this.collisionDistance].InnerText),
								0.5,
                                new Vector3 (Convert.ToDouble (xmlList [0] [this.externalForce].Attributes ["x"].Value),
                                    Convert.ToDouble (xmlList [0] [this.externalForce].Attributes ["y"].Value),
                                    Convert.ToDouble (xmlList [0] [this.externalForce].Attributes ["z"].Value)),
								Convert.ToInt32 (xmlList [0] [this.maxThreadNumber].InnerText));

			return simulationParameters;
		}

		#endregion

	}
}

