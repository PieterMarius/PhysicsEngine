using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;  
using Utility;
using SimulationObjectDefinition;
using MonoPhysicsEngine;
using CollisionEngine;
using LCPSolver;

namespace TestPhysics
{
	public class TestWindow: GameWindow
	{
		public TestWindow () : base (512, 512, new GraphicsMode (32, 24, 0, 4))
		{
		}

		Stopwatch stopwatch = new Stopwatch();

		int[] textureID;

		int[] displayList;

		double elapsedTime = 0.0;
		long performaceValue = 0;

		List<CollisionPointStructure> collPoint;

		SimulationObject[] simulationObjects;
		IConstraint[] simulationJoints;

		CollisionEngineParameters collisionEngineParameters;
		SolverParameters solverParameters;
		SimulationParameters simulationParameters;
		PhysicsEngine physicsEngine;

		bool pause = false;

		int redTexture;

		#region Keyboard and mouse variables

		OpenTK.Input.MouseState current, previous;
		float xrot = 0.0f;
		float yrot = 0.0f;
		float xpos = 0.0f;
		float ypos = 0.0f;
		float zpos = -7.0f;

		#endregion

		void initProgram()
		{
			try
			{
				//var env = new BuildEnvironment();
				//env.GetPhysicsEnvironment();

				//LoadObject loadObject = new LoadObject ("startJoint.xml");
				//LoadObject loadObject = new LoadObject ("configJoint.xml");
				//var loadObject = new LoadObject ("startConfig.xml");
				//var loadObject = new LoadObject ("carConfig.xml");
				var loadObject = new LoadObject("testJointBridge.xml");


				simulationObjects = loadObject.LoadSimulationObjects ();
				simulationJoints = loadObject.LoadSimulationJoints (simulationObjects);

				displayList = loadObject.GetOpenGLObjectList ();

				//Carico le texture
				textureID = loadObject.LoadTexture ();
				redTexture = OpenGLUtilities.LoadTexture ("red.bmp");

				//Set Collision Detection
				collisionEngineParameters = new CollisionEngineParameters();

				//Set Solver
				solverParameters = new SolverParameters();
				
				//Set Physics engine
				simulationParameters = new SimulationParameters();
				
				physicsEngine = new PhysicsEngine(
					simulationParameters,
					collisionEngineParameters,
					solverParameters);

				physicsEngine.SetSolver(SolverType.NonLinearConjugateGradient);

				for (int i = 0; i < simulationObjects.Count (); i++) 
				{
					physicsEngine.AddObject (simulationObjects [i]);
				}

				for (int i = 0; i < simulationJoints.Count (); i++) 
				{
					physicsEngine.AddJoint (simulationJoints [i]);
				}

				//var obj = physicsEngine.GetJointsList();
//
//				obj.Add(null);

				#region Object Removing

				//physicsEngine.RemoveAllJoints();

				//physicsEngine.RemoveAllObjects();

				//int objectIndex = 2;

				//physicsEngine.RemoveObject(objectIndex);

				////Graphics engine
				//var buf = displayList.ToList();
				//buf.RemoveAt(objectIndex);
				//displayList = buf.ToArray();

				//var buf1 = textureID.ToList();
				//buf1.RemoveAt(objectIndex);
				//textureID = buf1.ToArray();

				#endregion


				collPoint = new List<CollisionPointStructure> ();

			}
			catch (Exception e) 
			{
				throw new Exception (e.StackTrace);
			}
		}
			
		#region OpenGL Windows Settings

		protected override void OnLoad(EventArgs e)
		{
			base.OnLoad (e);

			initProgram ();
				
			Title = "Physics Window";

			//OpenGL Windows properties settings
			GL.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);

		}
			

		protected override void OnRenderFrame(FrameEventArgs e)
		{
			base.OnRenderFrame (e);

			GL.Clear (ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
			GL.Enable (EnableCap.DepthTest);
			GL.BlendFunc (BlendingFactorSrc.SrcAlpha, BlendingFactorDest.One);
			GL.DepthFunc (DepthFunction.Lequal);

			GL.LoadIdentity ();

			MoveCamera ();

//			displayOrigin ();
			displayContact ();
			displayJoint ();
			//displayVertex (0);
			//displayVertex (1);
			//displayVertex (2);

			for (int i = 0; i < physicsEngine.ObjectCount(); i++) 
				SetOpenGLObjectMatrix (i);
				
			GL.Flush ();
			SwapBuffers ();


		}

		protected override void OnUpdateFrame(FrameEventArgs e)
		{
			base.OnUpdateFrame (e);

			if (OpenTK.Input.Keyboard.GetState () [OpenTK.Input.Key.Escape])
				Exit ();

			GL.Viewport (0, 0, Width, Height);
			GL.MatrixMode (MatrixMode.Projection);
			GL.LoadIdentity ();

			OpenGLUtilities.GluPerspective (
				60, 
				Convert.ToDouble (Width) / Convert.ToDouble (Height), 
				1.0, 
				100.0);

			GL.MatrixMode (MatrixMode.Modelview);

			//Physics
			UpdateMouse ();
			UpdateKeyboard ();

			try {

				if (!pause)
				{
					stopwatch.Reset();
					stopwatch.Start();

					collPoint.Clear();

					physicsEngine.Simulate();
					//for (int i = 0; i < physicsEngine.JointsCount(); i++)
					//{
					//	physicsEngine.GetJoint(i).AddTorque(physicsEngine.GetSimulationObjectArray(), 0.0, 0.4);
					//}

					stopwatch.Stop();

					elapsedTime += 0.015;
					performaceValue += stopwatch.ElapsedMilliseconds;

					Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);
					Console.WriteLine("ElapsedTime " + elapsedTime + " performanceValue " + performaceValue);
					Console.WriteLine("Partial " + stopwatch.ElapsedMilliseconds);
					Console.WriteLine();

					//if (elapsedTime > 6.0)
					//	Exit();
				}
			} 
			catch (Exception ex) 
			{
				throw new Exception ("Physics engine error. " + ex.StackTrace);
			}
			collPoint = physicsEngine.GetCollisionPointStrucureList ();

		}
			
		#region TEST

		protected void setOrthographicProjection()
		{
			GL.MatrixMode (MatrixMode.Projection);

			GL.PushMatrix ();

			GL.LoadIdentity ();

		}

		#endregion

		#endregion

		protected void MoveCamera () 
		{
			GL.Rotate (xrot, 1.0, 0.0, 0.0);  //rotate 
			GL.Rotate (yrot, 0.0, 1.0, 0.0);  //rotate 
			GL.Translate (xpos, ypos, zpos); //translate the screen
		}
			
		#region Mouse Input

		public void UpdateMouse()
		{
			current = OpenTK.Input.Mouse.GetState ();
			if (current != previous) 
			{
				int xdelta = current.X - previous.X;
				int ydelta = current.Y - previous.Y;
				xrot += Convert.ToSingle (ydelta);
				yrot += Convert.ToSingle (xdelta);
			}
			previous = current;
		}

		#endregion

		#region Keyboard input

		int selectedObjIndex = -1;

		protected void UpdateKeyboard ()
		{
			if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.W])
			{
				float xrotrad, yrotrad;
				yrotrad = (yrot / 180 * 3.141592654f);
				xrotrad = (xrot / 180 * 3.141592654f); 
				xpos += Convert.ToSingle(Math.Sin(yrotrad)) * 0.1f;
				zpos -= Convert.ToSingle(Math.Cos(yrotrad)) * 0.1f;
				ypos -= Convert.ToSingle(Math.Sin(xrotrad)) * 0.1f;
			}
			if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.S])
			{
				float xrotrad, yrotrad;
				yrotrad = (yrot / 180 * 3.141592654f);
				xrotrad = (xrot / 180 * 3.141592654f); 
				xpos -= Convert.ToSingle(Math.Sin(yrotrad)) * 0.1f;
				zpos += Convert.ToSingle(Math.Cos(yrotrad)) * 0.1f;
				ypos += Convert.ToSingle(Math.Sin(xrotrad)) * 0.1f;
			}

			if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.D])
			{
				float yrotrad;
				yrotrad = (yrot / 180 * 3.141592654f);
				xpos += Convert.ToSingle(Math.Cos(yrotrad)) * 0.1f;
				zpos += Convert.ToSingle(Math.Sin(yrotrad)) * 0.1f;
			}

			if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.A])
			{
				float yrotrad;
				yrotrad = (yrot / 180 * 3.141592654f);
				xpos -= Convert.ToSingle(Math.Cos(yrotrad)) * 0.1f;
				zpos -= Convert.ToSingle(Math.Sin(yrotrad)) * 0.1f;
			}

			// pause
			if (OpenTK.Input.Keyboard.GetState () [OpenTK.Input.Key.P]) 
			{
				selectedObjIndex = -1;
				if (pause) {
					pause = false;
				} else {
					pause = true;
				}
			}

			if (OpenTK.Input.Keyboard.GetState () [OpenTK.Input.Key.R]) 
			{
				physicsEngine.Dispose();
				OnLoad(null);
			}

			if (OpenTK.Input.Keyboard.GetState () [OpenTK.Input.Key.F]) 
			{
				pause = true;
				selectedObjIndex = Convert.ToInt32 (Console.ReadLine ());
			}

			if (OpenTK.Input.Keyboard.GetState () [OpenTK.Input.Key.Down]) 
			{
				
			}

		}

		private void UpdateVertexPosition(int index)
		{
			simulationObjects[index].SetRotationMatrix (
				PhysicsEngineMathUtility.Quaternion.ConvertToMatrix (PhysicsEngineMathUtility.Quaternion.Normalize (simulationObjects[index].RotationStatus)));

			for (int i = 0; i < simulationObjects[index].ObjectGeometry.VertexPosition.Length; i++) 
			{
				PhysicsEngineMathUtility.Vector3 relPositionRotate = simulationObjects [index].RotationMatrix * simulationObjects [index].RelativePositions [i];
				simulationObjects [index].ObjectGeometry.SetVertexPosition (simulationObjects [index].Position + relPositionRotate, i);
			}
		}
			

		#endregion
			
		#region "Private Methods"

		private void SetOpenGLObjectMatrix(int id)
		{
			// TODO parte da modificare
			//Matrice da utilizzare come costante
			PhysicsEngineMathUtility.Matrix3x3 rotatioMatrix = physicsEngine.GetObject (id).RotationMatrix;
			PhysicsEngineMathUtility.Vector3 position = physicsEngine.GetObject (id).Position;
			ObjectType type = physicsEngine.GetObject (id).ObjectType;

			//if (type != ObjectType.JointConnector) {

				GL.PushMatrix ();
				GL.Enable (EnableCap.Texture2D);

				Matrix4 positionMatrix = new Matrix4 (
					                        Convert.ToSingle (rotatioMatrix.r1c1), 
					                        Convert.ToSingle (rotatioMatrix.r2c1), 
					                        Convert.ToSingle (rotatioMatrix.r3c1),
					                        0.0f,

					                        Convert.ToSingle (rotatioMatrix.r1c2), 
					                        Convert.ToSingle (rotatioMatrix.r2c2), 
					                        Convert.ToSingle (rotatioMatrix.r3c2),
					                        0.0f,

					                        Convert.ToSingle (rotatioMatrix.r1c3), 
					                        Convert.ToSingle (rotatioMatrix.r2c3), 
					                        Convert.ToSingle (rotatioMatrix.r3c3),
					                        0.0f,

					                        Convert.ToSingle (position.x), 
					                        Convert.ToSingle (position.y), 
					                        Convert.ToSingle (position.z),
					                        1.0f);
			
				Matrix4 mView = positionMatrix;

				var dmviewData = new float[] {
					mView.M11, mView.M12, mView.M13, mView.M14,
					mView.M21, mView.M22, mView.M23, mView.M24,
					mView.M31, mView.M32, mView.M33, mView.M34,
					mView.M41, mView.M42, mView.M43, mView.M44
				};

				//Fine parte da modificare
				GL.MultMatrix (dmviewData);

				//Inserire il textire ID
				if (id == selectedObjIndex)
					GL.BindTexture (TextureTarget.Texture2D, redTexture);
				else
					GL.BindTexture (TextureTarget.Texture2D, textureID [id]);
			
				GL.CallList (displayList [id]);
				GL.Disable (EnableCap.Texture2D);


	
				GL.PopMatrix ();
			//}
		}

		private void displayContact()
		{
			for (int i = 0; i < collPoint.Count; i++) {
				for (int j = 0; j < collPoint [i].CollisionPoints.Length; j++) {

					GL.PushMatrix ();

					Matrix4 mView = Matrix4.CreateTranslation (
						Convert.ToSingle (collPoint[i].CollisionPoints [j].CollisionPointA.x), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].CollisionPointA.y), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].CollisionPointA.z));

					var dmviewData = new float[] {
						mView.M11, mView.M12, mView.M13, mView.M14,
						mView.M21, mView.M22, mView.M23, mView.M24,
						mView.M31, mView.M32, mView.M33, mView.M34,
						mView.M41, mView.M42, mView.M43, mView.M44
					};

					GL.MultMatrix (dmviewData);

					//GL.Color3 (1.0f, 0.0, 0.0);
					OpenGLUtilities.drawSolidCube (0.08f);

					GL.PopMatrix ();

					GL.PushMatrix ();

					Matrix4 mView1 = Matrix4.CreateTranslation (
						Convert.ToSingle (collPoint[i].CollisionPoints [j].CollisionPointB.x), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].CollisionPointB.y), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].CollisionPointB.z));

					var dmviewData1 = new float[] {
						mView1.M11, mView1.M12, mView1.M13, mView1.M14,
						mView1.M21, mView1.M22, mView1.M23, mView1.M24,
						mView1.M31, mView1.M32, mView1.M33, mView1.M34,
						mView1.M41, mView1.M42, mView1.M43, mView1.M44
					};

					GL.MultMatrix (dmviewData1);


					//GL.Color3 (1.0f, 0.0, 0.0);
					OpenGLUtilities.drawSolidCube (0.08f);

					GL.PopMatrix ();

				}
			}
		}

		private void displayVertex(int index)
		{
			for (int i = 0; i < physicsEngine.GetObject (index).ObjectGeometry.VertexPosition.Length; i++) 
			{
				GL.PushMatrix ();

				Matrix4 mView = Matrix4.CreateTranslation (
					Convert.ToSingle (physicsEngine.GetObject (index).ObjectGeometry.VertexPosition[i].x), 
					Convert.ToSingle (physicsEngine.GetObject (index).ObjectGeometry.VertexPosition[i].y), 
					Convert.ToSingle (physicsEngine.GetObject (index).ObjectGeometry.VertexPosition[i].z));

				var dmviewData = new float[] {
					mView.M11, mView.M12, mView.M13, mView.M14,
					mView.M21, mView.M22, mView.M23, mView.M24,
					mView.M31, mView.M32, mView.M33, mView.M34,
					mView.M41, mView.M42, mView.M43, mView.M44
				};

				GL.MultMatrix (dmviewData);

				OpenGLUtilities.drawSolidCube (0.08f);

				GL.PopMatrix ();
			}
		}

		private void displayJoint()
		{
			

			for (int i = 0; i < physicsEngine.JointsCount(); i++) 
			{
					GL.PushMatrix ();

				IConstraint joint = physicsEngine.GetJoint(i);

					Matrix4 mView = Matrix4.CreateTranslation (
						                Convert.ToSingle (joint.GetAnchorPosition ().x), 
						                Convert.ToSingle (joint.GetAnchorPosition ().y), 
						                Convert.ToSingle (joint.GetAnchorPosition ().z));

					var dmviewData = new float[] {
						mView.M11, mView.M12, mView.M13, mView.M14,
						mView.M21, mView.M22, mView.M23, mView.M24,
						mView.M31, mView.M32, mView.M33, mView.M34,
						mView.M41, mView.M42, mView.M43, mView.M44
					};

					GL.MultMatrix (dmviewData);

					OpenGLUtilities.drawSolidCube (0.08f);

					GL.PopMatrix ();

			}
		}

		private void displayOrigin()
		{
			GL.PushMatrix ();

			Matrix4 mView = Matrix4.CreateTranslation (
				Convert.ToSingle (0.0), 
				Convert.ToSingle (0.0), 
				Convert.ToSingle (0.0));

			var dmviewData = new float[] {
				mView.M11, mView.M12, mView.M13, mView.M14,
				mView.M21, mView.M22, mView.M23, mView.M24,
				mView.M31, mView.M32, mView.M33, mView.M34,
				mView.M41, mView.M42, mView.M43, mView.M44
			};

			GL.MultMatrix (dmviewData);

			OpenGLUtilities.drawSolidCube (0.10f);

			GL.PopMatrix ();
		}


		#endregion



//		protected override void OnResize(EventArgs e)
//		{
//			base.OnResize (e);
//			GL.Viewport (ClientRectangle.X, 
//				ClientRectangle.Y, 
//				ClientRectangle.Width, 
//				ClientRectangle.Height);
//
//			GL.MatrixMode (MatrixMode.Projection);
//			openGLUtilities.gluPerspective (
//				60, 
//				Convert.ToDouble(Width) / Convert.ToDouble(Height), 
//				1.0, 
//				100.0);
//		}
	}
}

