using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using System.Drawing.Imaging;
using System.Diagnostics;
using OpenTK;
using OpenTK.Input;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using ObjLoader;
using ObjLoader.Loader.Loaders;  
using Utility;
using SimulationObjectDefinition;
using MonoPhysicsEngine;
using CollisionEngine;
using LCPSolver;
using Loading;

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

		List<CollisionPointStructure> collPoint;

		SimulationObject[] simulationObjects;
		ObjectConstraint[] simulationJoints;

		CollisionEngineParameters collisionEngineParameters;
		SolverParameters solverParameters;
		SimulationParameters simulationParameters;
		PhysicsEngine physicsEngine;

		ICollisionEngine collisionEngine;
		ISolver lcpSolver;

		int count = 0;
		bool pause = false;

		int redTexture;

		//ObjectGeometry[] objectsGeometry;

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



			//LoadObject loadObject = new LoadObject ("startJoint.xml");
			LoadObject loadObject = new LoadObject ("configJoint.xml");
			//LoadObject loadObject = new LoadObject ("startConfig.xml");
			//LoadObject loadObject = new LoadObject ("carConfig.xml");

			simulationObjects = loadObject.LoadSimulationObjects ();
			simulationJoints = loadObject.LoadSimulationJoints (simulationObjects);

			displayList = loadObject.GetOpenGLObjectList ();

			//Carico le texture
			textureID = loadObject.LoadTexture ();
			redTexture = OpenGLUtilities.LoadTexture ("red.bmp");


			//Set Collision Detection
			this.collisionEngineParameters = new CollisionEngineParameters();
			this.collisionEngine = new CollisionDetectionEngine(
				this.collisionEngineParameters);

			//Set Solver
			this.solverParameters = new SolverParameters();
			this.lcpSolver = new LCPSolver.GaussSeidel (this.solverParameters);
			//this.lcpSolver = new LCPSolver.NonLinearConjugateGradient (this.solverParameters);

			//Set Physics engine
			this.simulationParameters = new SimulationParameters();
//			LoadEngineConfig engineConfig = new LoadEngineConfig ("startConfig.xml");
//			this.simulationParameters = engineConfig.ReadEngineConfig ();
//
			this.simulationParameters.SetExternalForce (new PhysicsEngineMathUtility.Vector3 (0.0, -1.2, 0.0));
			IContactPartitioningEngine contactPartitionEngine = new ContactPartitioningEngine ();
			this.physicsEngine = new PhysicsEngine (
				this.collisionEngine,
				this.lcpSolver,
				contactPartitionEngine,
				this.simulationParameters);

			for (int i = 0; i < this.simulationObjects.Count (); i++) 
			{
				this.physicsEngine.AddObject (this.simulationObjects [i]);
			}

			for (int i = 0; i < this.simulationJoints.Count (); i++) 
			{
				this.physicsEngine.AddJoint (this.simulationJoints [i]);
			}

			this.collPoint = new List<CollisionPointStructure> ();

			}
			catch (Exception e) 
			{
				throw new Exception (e.StackTrace);
			}

			/* TODO Fine */

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

			for (int i = 0; i < displayList.Length; i++) 
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
			this.UpdateMouse ();
			this.UpdateKeyboard ();

			try {

			if (!pause){
				stopwatch.Reset ();
				stopwatch.Start ();

				collPoint.Clear ();

				this.physicsEngine.Simulate (null);

				//DEBUG
//				count++;
//				Console.WriteLine ("Solver Error: " + this.physicsEngine.GetSolverError ());
//				Console.WriteLine ("Position:" + this.physicsEngine.GetObject (0).Position.x + " ; " +
//					this.physicsEngine.GetObject (0).Position.y + " ; " +
//					this.physicsEngine.GetObject (0).Position.z);
//				Console.WriteLine ("Velocity:" + this.physicsEngine.GetObject (0).LinearVelocity.x + " ; " +
//					this.physicsEngine.GetObject (0).LinearVelocity.y + " ; " +
//					this.physicsEngine.GetObject (0).LinearVelocity.z);

				stopwatch.Stop ();

				Console.WriteLine("Engine Elapsed={0}",stopwatch.ElapsedMilliseconds);
				Console.WriteLine ();
			}
			}catch (Exception) {
				throw new Exception ("Physics engine error.");
			}
			collPoint = this.physicsEngine.GetCollisionPointStrucureList ();

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

		//Test collisione
		double angle = 0.0;

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
				initProgram ();
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
			PhysicsEngineMathUtility.Matrix3x3 rotatioMatrix = this.physicsEngine.GetObject (id).RotationMatrix;
			PhysicsEngineMathUtility.Vector3 position = this.physicsEngine.GetObject (id).Position;
			ObjectType type = this.physicsEngine.GetObject (id).ObjectType;

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

				float[] dmviewData = new float[] {
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
						Convert.ToSingle (collPoint[i].CollisionPoints [j].collisionPointA.x), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].collisionPointA.y), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].collisionPointA.z));

					float[] dmviewData = new float[] {
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
						Convert.ToSingle (collPoint[i].CollisionPoints [j].collisionPointB.x), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].collisionPointB.y), 
						Convert.ToSingle (collPoint[i].CollisionPoints [j].collisionPointB.z));

					float[] dmviewData1 = new float[] {
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
			for (int i = 0; i < this.physicsEngine.GetObject (index).ObjectGeometry.VertexPosition.Length; i++) 
			{
				GL.PushMatrix ();

				Matrix4 mView = Matrix4.CreateTranslation (
					Convert.ToSingle (this.physicsEngine.GetObject (index).ObjectGeometry.VertexPosition[i].x), 
					Convert.ToSingle (this.physicsEngine.GetObject (index).ObjectGeometry.VertexPosition[i].y), 
					Convert.ToSingle (this.physicsEngine.GetObject (index).ObjectGeometry.VertexPosition[i].z));

				float[] dmviewData = new float[] {
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
			List<ObjectConstraint> jointList = this.physicsEngine.GetJointsList ();

			for (int i = 0; i < jointList.Count; i++) 
			{
				for (int j = 0; j < jointList [i].ConstraintList.Length; j++) {

					GL.PushMatrix ();

					Matrix4 mView = Matrix4.CreateTranslation (
						                Convert.ToSingle (jointList [i].ConstraintList [j].GetAnchorPosition ().x), 
						                Convert.ToSingle (jointList [i].ConstraintList [j].GetAnchorPosition ().y), 
						                Convert.ToSingle (jointList [i].ConstraintList [j].GetAnchorPosition ().z));

					float[] dmviewData = new float[] {
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
		}

		private void displayOrigin()
		{
			GL.PushMatrix ();

			Matrix4 mView = Matrix4.CreateTranslation (
				Convert.ToSingle (0.0), 
				Convert.ToSingle (0.0), 
				Convert.ToSingle (0.0));

			float[] dmviewData = new float[] {
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

