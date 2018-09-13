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
using SharpPhysicsEngine;
using SharpEngineMathUtility;

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

		public PhysicsEngineParameters ReadEngineConfig()
		{
			XmlDocument xmlDoc = new XmlDocument();
			xmlDoc.Load(this.FileNameEngineProperties);

			XmlNodeList xmlList = xmlDoc.SelectNodes(nodePathObjects);

			PhysicsEngineParameters simulationParameters = new PhysicsEngineParameters (
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
                                new Vector3d (Convert.ToDouble (xmlList [0] [this.externalForce].Attributes ["x"].Value),
                                    Convert.ToDouble (xmlList [0] [this.externalForce].Attributes ["y"].Value),
                                    Convert.ToDouble (xmlList [0] [this.externalForce].Attributes ["z"].Value)),
								Convert.ToInt32 (xmlList [0] [this.maxThreadNumber].InnerText));

			return simulationParameters;
		}

		#endregion

	}
}

