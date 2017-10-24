using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using Utility;
using SharpPhysicsEngine.ShapeDefinition;
using SharpPhysicsEngine;
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.NonConvexDecomposition.Octree;
using SharpEngineMathUtility;
using ConvexHullGenerator;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;

namespace TestPhysics
{
	public class TestWindow: GameWindow
	{
		public TestWindow () : base (512, 512, new GraphicsMode (32, 24, 0, 4))
		{
		}

		Stopwatch stopwatch = new Stopwatch();

		int[][] textureID;

		int[][] displayList;

		double elapsedTime = 0.0;
		long performaceValue = 0;

		List<CollisionPointStructure> collPoint;
		List<List<CollisionPointStructure>> collisionPartitionedPoints;

		IShape[] simulationObjects;
		IConstraint[] simulationJoints;

		CollisionEngineParameters collisionEngineParameters;
		SolverParameters solverParameters;
		PhysicsEngineParameters simulationParameters;
		SharpPhysicsEngine.SharpEngine physicsEngine;

		List<List<double>> colorList = new List<List<double>>();

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

		void TestNumeric()
		{
			int nvalue = 30000000;
			stopwatch.Reset();
			stopwatch.Start();
			
			System.Numerics.Vector3[] testVector = new System.Numerics.Vector3[nvalue];

			//double[] vv = new double[] { 0.0, 0.0, 0.0, 0.0 };
			for (int i = 0; i < nvalue; i++)
				testVector[i] = new System.Numerics.Vector3(2, 3, 5);

			System.Numerics.Vector3 test;
			for (int i = 0; i < nvalue; i++)
				test = testVector[i] * 2.0f;

			stopwatch.Stop();
			Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

			stopwatch.Reset();
			stopwatch.Start();

			SharpEngineMathUtility.Vector3[] testVector1 = new SharpEngineMathUtility.Vector3[nvalue];

			for (int i = 0; i < nvalue; i++)
				testVector1[i] = new SharpEngineMathUtility.Vector3(2.0, 3.0, 5.0);

			SharpEngineMathUtility.Vector3 test1;
			for (int i = 0; i < nvalue; i++)
				test1 = testVector1[i] * 2.0;

			stopwatch.Stop();
			Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

			stopwatch.Reset();
			stopwatch.Start();

			//System.Numerics.Vector<double>[] testVector2 = new System.Numerics.Vector<double>[nvalue];

			for (int i = 0; i < nvalue; i++)
				testVector1[i] = new SharpEngineMathUtility.Vector3(2.0, 3.0, 5.0);

			//{ new SharpEngineMathUtility.Vector3(0.0, 0.0, 0.0) };

			double[] test2;
			//System.Numerics.Vector<double> tt = new System.Numerics.Vector<double>(new SharpEngineMathUtility.Vector3(1.0, 0.0, 0.0));
			List<double> test3 = new List<double>();
			//double[] test4 = new double[nvalue * 3];

			for (int i = 0; i < nvalue; i++)
			{
			   // int index = (i * 3);
				test3.AddRange(testVector1[i].ToList);
				//test4[index] = testVector1[i].x;
				//test4[index +1] = testVector1[i].y;
				//test4[index +2] = testVector1[i].z;

				//test3.AddRange(testVector1[i].Array);
				//test3.Add(0);


			}

			test2 = SIMDMathUtility.SIMDArrayProductScalar(test3.ToArray(), 2.0);

			stopwatch.Stop();
			Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);


		}

		NonConvexSphereDecomposition testConvexDecomp = new NonConvexSphereDecomposition();
		List<ShapeDecompositionOutput> convexShape = null;

		AABB region = null;


		ShapeConvexDecomposition shapeConvexDec = null;


		List<Line> octTreeLine = new List<Line>();
		void initProgram()
		{
			//TestNumeric();
			try
			{

                //testConvexDecomp.Decompose(0.08);

                //TestNumeric();

                //env.GetPhysicsEnvironment();




                //var env = new BuildEnvironment();


                ////AddSoftBody();



                //physicsEngine = env.GetPhysicsEnvironment();
                //displayList = env.GetOpenGLEnvironment();
                //textureID = env.LoadTexture();


                //ISoftShape softShape = physicsEngine.GetShape(3) as SoftShape;

                //stopwatch.Reset();
                //stopwatch.Start();

                //region = softShape.AABBox;



                //shapeConvexDec = new ShapeConvexDecomposition(
                //region,
                //softShape.Triangle);

                //convexShape = shapeConvexDec.GetConvexShapeList(
                //	Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position, item.TriangleIndex.ToArray(),0)),
                //	0.2);

                //AABB testRegion = new AABB(
                //new SharpEngineMathUtility.Vector3(-0.5, -0.5, 0.6),
                //new SharpEngineMathUtility.Vector3(0.5, 0.5, 1.6));

                //convexShape = shapeConvexDec.GetIntersectedShape(testRegion,
                //	Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position,
                //			item.TriangleIndex.ToArray(),0)),
                //		0.2);

                //stopwatch.Stop();
                //Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);


                //OctTree octTree = new OctTree(region, softShape.Triangle.ToList(), Array.ConvertAll(softShape.ShapePoints, item => new SharpEngineMathUtility.Vector3(item.Position)));
                //octTree.BuildOctree();
                //octTree.Render(ref octTreeLine);

                // octTree.GetConvexShapeList((IShape)softShape, ref convexShape);



                //var obj = physicsEngine.GetJointsList();
                //
                //				obj.Add(null);

                //Exit();

                //LoadEngineByXml();
                LoadEngineByBuilder();


            }
			catch (Exception e) 
			{
				throw new Exception (e.StackTrace);
			}
		}

        private void LoadEngineByBuilder()
        {
            var env = new BuildEnvironment();


            //AddSoftBody();
            

            physicsEngine = env.GetPhysicsEnvironment();
            displayList = env.GetOpenGLEnvironment();
            textureID = env.LoadTexture();

            collPoint = new List<CollisionPointStructure>();
            collisionPartitionedPoints = new List<List<CollisionPointStructure>>();
        }

        private void LoadEngineByXml()
        {
            //LoadObject loadObject = new LoadObject("startJoint.xml");
            //LoadObject loadObject = new LoadObject("configJoint.xml");
            //var loadObject = new LoadObject("startConfig.xml");
            //var loadObject = new LoadObject("carConfig.xml");
            var loadObject = new LoadObject("testJointBridge.xml");
            //var loadObject = new LoadObject("compositeObjectConfig.xml");
            //var loadObject = new LoadObject("frictionTestConfig.xml");

            //var loadObject = new LoadObject("softBodyConfig.xml");


            simulationObjects = loadObject.LoadSimulationObjects();
            simulationJoints = loadObject.LoadSimulationJoints(simulationObjects);
            
            displayList = loadObject.GetOpenGLObjectList();
                        
            ////Carico le texture
            textureID = loadObject.LoadTexture();
            redTexture = OpenGLUtilities.LoadTexture("red.bmp");

            physicsEngine = new SharpEngine();

            physicsEngine.SetSolver(SolverType.ProjectedGaussSeidel);

            for (int i = 0; i < simulationObjects.Count(); i++)
            {
                physicsEngine.AddShape(simulationObjects[i]);
            }

            for (int i = 0; i < simulationJoints.Count(); i++)
            {
                physicsEngine.AddJoint(simulationJoints[i]);
            }

            //physicsEngine.AddShape(AddSoftBody());

            pause = true;

            collPoint = new List<CollisionPointStructure>();
            collisionPartitionedPoints = new List<List<CollisionPointStructure>>();

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

			//displayOrigin ();
			//displayContact ();
			//displayBaseContact();
			//displayJoint ();
			//displaySphere(testConvexDecomp.basePoint);
			//DisplayObject();

			displayPartitionedContact();

			//for (int i = 0; i < physicsEngine.ObjectCount(); i++)
			//    displayVertex(i);

			//displayAABB();
			//displayVertex (0);
			//displayVertex (1);
			//displayVertex (2);

			for (int i = 0; i < physicsEngine.ShapesCount(); i++)
				SetOpenGLObjectMatrixAndDisplayObject(i);

			//displayOctree();
			//displayConvexDecomposition();

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
					//    physicsEngine.GetJoints(i).AddTorque(physicsEngine.GetSimulationObjectArray(), 0.0, 0.4);
					//}

					stopwatch.Stop();

					elapsedTime += 0.015;
					performaceValue += stopwatch.ElapsedMilliseconds;

					Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);
					Console.WriteLine("ElapsedTime " + elapsedTime + " performanceValue " + performaceValue);
					Console.WriteLine("Partial " + stopwatch.ElapsedMilliseconds);
					Console.WriteLine();



					pause = true;
					//if (elapsedTime > 6.0)
					//	Exit();
					//collPoint = physicsEngine.GetCollisionPointStrucureList();
					//collisionPartitionedPoints = physicsEngine.GetPartitionedCollisionPoints();

					//colorList = new List<List<double>>();
					//if (collisionPartitionedPoints != null)
					//{
					//    for (int i = 0; i < collisionPartitionedPoints.Count; i++)
					//    {
					//        List<double> color = new List<double>();
					//        color.Add(GetRandomNumber(0.0, 1.0));
					//        color.Add(GetRandomNumber(0.0, 1.0));
					//        color.Add(GetRandomNumber(0.0, 1.0));
					//        colorList.Add(color);
					//    }
					//}
				}
			} 
			catch (Exception ex) 
			{
				throw new Exception ("Physics engine error. " + ex.StackTrace);
			}

			
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
			current = OpenTK.Input.Mouse.GetCursorState();
			
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

		//private void UpdateVertexPosition(int index)
		//{
		//	simulationObjects[index].SetRotationMatrix (
		//		PhysicsEngineMathUtility.Quaternion.ConvertToMatrix (PhysicsEngineMathUtility.Quaternion.Normalize (simulationObjects[index].RotationStatus)));

		//	for (int i = 0; i < simulationObjects[index].ObjectGeometry[0].VertexPosition.Length; i++) 
		//	{
		//		PhysicsEngineMathUtility.Vector3 relPositionRotate = simulationObjects [index].RotationMatrix * simulationObjects [index].RelativePositions [i];
		//		simulationObjects [index].ObjectGeometry[0].SetVertexPosition (simulationObjects [index].Position + relPositionRotate, i);
		//	}
		//}
			

		#endregion
			
		#region "Private Methods"

		//private void DisplayObject()
		//{
		//    for (int i = 0; i < displayList.Length; i++)
		//    {
		//        for (int j = 0; j < displayList[i].Length; j++)
		//        {
		//            GL.PushMatrix();
		//            GL.Enable(EnableCap.Texture2D);

		//            GL.BindTexture(TextureTarget.Texture2D, textureID[id][i]);

		//            GL.CallList(displayList[i][j]);
		//            GL.Disable(EnableCap.Texture2D);

		//            GL.PopMatrix();
		//        }
		//    }
		//}

		private void SetOpenGLObjectMatrixAndDisplayObject(int id)
		{
			// TODO parte da modificare
			//Matrice da utilizzare come costante
			IShape shape = physicsEngine.GetShapes()[id];

			ISoftShape softShape = shape as ISoftShape;
			if (softShape != null)
				DisplaySoftPoint(softShape);
			else
			{
				SharpEngineMathUtility.Matrix3x3 rotatioMatrix = shape.RotationMatrix;
				SharpEngineMathUtility.Vector3 position = shape.Position;

				ObjectType type = shape.ObjectType;
				
				for (int i = 0; i < SharpPhysicsEngine.ShapeDefinition.Helper.GetGeometry(shape).Length; i++)
				{
					GL.PushMatrix();
					GL.Enable(EnableCap.Texture2D);

					SharpEngineMathUtility.Vector3 compoundPos = new SharpEngineMathUtility.Vector3();

					if (shape is ICompoundShape)
						compoundPos = ((ICompoundShape)shape).StartCompoundPositionObjects[i];

					SharpEngineMathUtility.Vector3 positionMt = position + compoundPos -
																  shape.StartPosition;

					Matrix4 positionMatrix = new Matrix4(
												Convert.ToSingle(rotatioMatrix.r1c1),
												Convert.ToSingle(rotatioMatrix.r2c1),
												Convert.ToSingle(rotatioMatrix.r3c1),
												0.0f,

												Convert.ToSingle(rotatioMatrix.r1c2),
												Convert.ToSingle(rotatioMatrix.r2c2),
												Convert.ToSingle(rotatioMatrix.r3c2),
												0.0f,

												Convert.ToSingle(rotatioMatrix.r1c3),
												Convert.ToSingle(rotatioMatrix.r2c3),
												Convert.ToSingle(rotatioMatrix.r3c3),
												0.0f,

												0.0f,
												0.0f,
												0.0f,
												1.0f);

					Matrix4 mView = positionMatrix;

					var dmviewData = new float[] {
					mView.M11, mView.M12, mView.M13, mView.M14,
					mView.M21, mView.M22, mView.M23, mView.M24,
					mView.M31, mView.M32, mView.M33, mView.M34,
					mView.M41, mView.M42, mView.M43, mView.M44
				};

					//Traslo sull'origine
					GL.Translate(position.x, position.y, position.z);
					//Ruoto
					GL.MultMatrix(dmviewData);
					//Traslo nella posizione desiderata
					GL.Translate(positionMt.x - position.x, positionMt.y - position.y, positionMt.z - position.z);


					//Inserire il textire ID
					if (id == selectedObjIndex)
						GL.BindTexture(TextureTarget.Texture2D, redTexture);
					else
						GL.BindTexture(TextureTarget.Texture2D, textureID[id][i]);

					GL.CallList(displayList[id][i]);
					GL.Disable(EnableCap.Texture2D);

					GL.PopMatrix();
				}
			}
					
		}

		private void DisplaySoftPoint(ISoftShape softShape)
		{
            GL.BindTexture(TextureTarget.Texture2D, textureID[3][0]);

            GL.CallList(displayList[3][0]);
            GL.Disable(EnableCap.Texture2D);

            GL.PopMatrix();

            foreach (var item in softShape.ShapePoints)
            {
                SharpEngineMathUtility.Vector3 relativePosition = item.Position;

                GL.PushMatrix();

                Matrix4 mView = Matrix4.CreateTranslation(
                    Convert.ToSingle(relativePosition.x),
                    Convert.ToSingle(relativePosition.y),
                    Convert.ToSingle(relativePosition.z));

                var dmviewData = new float[] {
                    mView.M11, mView.M12, mView.M13, mView.M14,
                    mView.M21, mView.M22, mView.M23, mView.M24,
                    mView.M31, mView.M32, mView.M33, mView.M34,
                    mView.M41, mView.M42, mView.M43, mView.M44
                };

                GL.MultMatrix(dmviewData);


                OpenGLUtilities.drawSolidCube(0.04f);

                GL.PopMatrix();
            }
        }

		private void displayContact()
		{
			for (int i = 0; i < collPoint.Count; i++)
			{
				for (int h = 0; h < collPoint[i].CollisionPointBase.Length; h++)
				{
					for (int j = 0; j < collPoint[i].CollisionPointBase[h].CollisionPoints.Length; j++)
					{

						GL.PushMatrix();

						Matrix4 mView = Matrix4.CreateTranslation(
							Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.Vertex.x),
							Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.Vertex.y),
							Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.Vertex.z));

						var dmviewData = new float[] {
						mView.M11, mView.M12, mView.M13, mView.M14,
						mView.M21, mView.M22, mView.M23, mView.M24,
						mView.M31, mView.M32, mView.M33, mView.M34,
						mView.M41, mView.M42, mView.M43, mView.M44
					};

						GL.MultMatrix(dmviewData);

						//GL.Color3 (1.0f, 0.0, 0.0);
						OpenGLUtilities.drawSolidCube(0.08f);

						GL.PopMatrix();

						GL.PushMatrix();

						Matrix4 mView1 = Matrix4.CreateTranslation(
							Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.Vertex.x),
							Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.Vertex.y),
							Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.Vertex.z));

						var dmviewData1 = new float[] {
						mView1.M11, mView1.M12, mView1.M13, mView1.M14,
						mView1.M21, mView1.M22, mView1.M23, mView1.M24,
						mView1.M31, mView1.M32, mView1.M33, mView1.M34,
						mView1.M41, mView1.M42, mView1.M43, mView1.M44
					};

						GL.MultMatrix(dmviewData1);


						//GL.Color3 (1.0f, 0.0, 0.0);
						OpenGLUtilities.drawSolidCube(0.08f);

						GL.PopMatrix();

					}
				}
			}
		}

		Random random = new Random();

		public double GetRandomNumber(double minimum, double maximum)
		{
			
			return random.NextDouble() * (maximum - minimum) + minimum;
		}

		
		private void displayPartitionedContact()
		{
			//List<List<CollisionPointStructure>> collisionPartitionedPoints = physicsEngine.GetPartitionedCollisionPoints();

			double[] color = new double[3];
			if (collisionPartitionedPoints != null)
			{
				for (int h = 0; h < collisionPartitionedPoints.Count; h++)
				{
					color[0] = colorList[h][0]; color[1] = colorList[h][1]; color[2] = colorList[h][2];
					List<CollisionPointStructure> collPointStr = collisionPartitionedPoints[h];

					for (int i = 0; i < collPointStr.Count; i++)
					{
						for (int n = 0; n < collPointStr[i].CollisionPointBase.Length; n++)
						{
							for (int j = 0; j < collPointStr[i].CollisionPointBase[n].CollisionPoints.Length; j++)
							{

								GL.PushMatrix();

								Matrix4 mView = Matrix4.CreateTranslation(
									Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointA.Vertex.x),
									Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointA.Vertex.y),
									Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointA.Vertex.z));

								var dmviewData = new float[] {
									mView.M11, mView.M12, mView.M13, mView.M14,
									mView.M21, mView.M22, mView.M23, mView.M24,
									mView.M31, mView.M32, mView.M33, mView.M34,
									mView.M41, mView.M42, mView.M43, mView.M44
									};

								GL.MultMatrix(dmviewData);

								GL.Color3(color);
								OpenGLUtilities.drawSolidCube(0.08f);
								GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);

								GL.PopMatrix();

								GL.PushMatrix();

								Matrix4 mView1 = Matrix4.CreateTranslation(
									Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointB.Vertex.x),
									Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointB.Vertex.y),
									Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointB.Vertex.z));

								var dmviewData1 = new float[] {
									mView1.M11, mView1.M12, mView1.M13, mView1.M14,
									mView1.M21, mView1.M22, mView1.M23, mView1.M24,
									mView1.M31, mView1.M32, mView1.M33, mView1.M34,
									mView1.M41, mView1.M42, mView1.M43, mView1.M44
									};

								GL.MultMatrix(dmviewData1);

								GL.Color3(color);

								OpenGLUtilities.drawSolidCube(0.08f);
								GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);

								GL.PopMatrix();

							}
						}
					}
				}
			}
		}

		private void displayBaseContact()
		{
			for (int i = 0; i < collPoint.Count; i++)
			{
				for (int n = 0; n < collPoint[i].CollisionPointBase.Length; n++)
				{
					GL.PushMatrix();

					Matrix4 mView = Matrix4.CreateTranslation(
						Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointA.Vertex.x),
						Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointA.Vertex.y),
						Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointA.Vertex.z));

					var dmviewData = new float[] {
					mView.M11, mView.M12, mView.M13, mView.M14,
					mView.M21, mView.M22, mView.M23, mView.M24,
					mView.M31, mView.M32, mView.M33, mView.M34,
					mView.M41, mView.M42, mView.M43, mView.M44
				};

					GL.MultMatrix(dmviewData);

					//GL.Color3 (1.0f, 0.0, 0.0);
					OpenGLUtilities.drawSolidCube(0.08f);

					GL.PopMatrix();

					GL.PushMatrix();

					Matrix4 mView1 = Matrix4.CreateTranslation(
						Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointB.Vertex.x),
						Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointB.Vertex.y),
						Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointB.Vertex.z));

					var dmviewData1 = new float[] {
					mView1.M11, mView1.M12, mView1.M13, mView1.M14,
					mView1.M21, mView1.M22, mView1.M23, mView1.M24,
					mView1.M31, mView1.M32, mView1.M33, mView1.M34,
					mView1.M41, mView1.M42, mView1.M43, mView1.M44
				};

					GL.MultMatrix(dmviewData1);


					OpenGLUtilities.drawSolidCube(0.08f);


					GL.PopMatrix();
				}

			}
		}

		private void displayVertex(int index)
		{

			for (int i = 0; i < SharpPhysicsEngine.ShapeDefinition.Helper.GetGeometry(physicsEngine.GetShape (index)).Length; i++) 
			{
				for (int j = 0; j < SharpPhysicsEngine.ShapeDefinition.Helper.GetGeometry(physicsEngine.GetShape(index))[i].RelativePosition.Length; j++)
				{
					SharpEngineMathUtility.Vector3 relativePosition = physicsEngine.GetShape(index).Position +
									(physicsEngine.GetShape(index).RotationMatrix * SharpPhysicsEngine.ShapeDefinition.Helper.GetGeometry(physicsEngine.GetShape(index))[i].RelativePosition[j]);

					GL.PushMatrix();

					Matrix4 mView = Matrix4.CreateTranslation(
						Convert.ToSingle(relativePosition.x),
						Convert.ToSingle(relativePosition.y),
						Convert.ToSingle(relativePosition.z));

					var dmviewData = new float[] {
					mView.M11, mView.M12, mView.M13, mView.M14,
					mView.M21, mView.M22, mView.M23, mView.M24,
					mView.M31, mView.M32, mView.M33, mView.M34,
					mView.M41, mView.M42, mView.M43, mView.M44
				};

					GL.MultMatrix(dmviewData);

					OpenGLUtilities.drawSolidCube(0.04f);

					GL.PopMatrix();
				}
			}
		}

		private void displayJoint()
		{
			for (int i = 0; i < physicsEngine.JointsCount(); i++) 
			{
					GL.PushMatrix ();

				IConstraint joint = physicsEngine.GetJoints(i);

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

		private void displayAABB()
		{
			IShape[] simObj = physicsEngine.GetShapes();
			for (int i = 0; i < simObj.Length; i++)
			{
				AABB joint = SharpPhysicsEngine.ShapeDefinition.Helper.GetGeometry(simObj[i])[0].AABBox;
								
				GL.PushMatrix();

				Matrix4 mView = Matrix4.CreateTranslation(
									Convert.ToSingle(joint.Max[0]),
									Convert.ToSingle(joint.Max[1]),
									Convert.ToSingle(joint.Max[2]));

				var dmviewData = new float[] {
					mView.M11, mView.M12, mView.M13, mView.M14,
					mView.M21, mView.M22, mView.M23, mView.M24,
					mView.M31, mView.M32, mView.M33, mView.M34,
					mView.M41, mView.M42, mView.M43, mView.M44
				};

				GL.MultMatrix(dmviewData);

				OpenGLUtilities.drawSolidCube(0.08f);

				GL.PopMatrix();


				GL.PushMatrix();

				mView = Matrix4.CreateTranslation(
									Convert.ToSingle(joint.Min[0]),
									Convert.ToSingle(joint.Min[1]),
									Convert.ToSingle(joint.Min[2]));

				dmviewData = new float[] {
						mView.M11, mView.M12, mView.M13, mView.M14,
						mView.M21, mView.M22, mView.M23, mView.M24,
						mView.M31, mView.M32, mView.M33, mView.M34,
						mView.M41, mView.M42, mView.M43, mView.M44
					};

				GL.MultMatrix(dmviewData);

				OpenGLUtilities.drawSolidCube(0.08f);

				GL.PopMatrix();

			}
		}

		private void displayOctree()
		{
			foreach (var item in octTreeLine)
				OpenGLUtilities.DrawLine(item.a, item.b);
		}

		private void displayConvexDecomposition()
		{
			ISoftShape softShape = physicsEngine.GetShape(3) as SoftShape;

			//region.Min = region.Min + new SharpEngineMathUtility.Vector3(-1.0, -1.0, -1.0);
			//region.Max = region.Max + new SharpEngineMathUtility.Vector3(1.0, 1.0, 1.0);

			//List<List<Vertex3Index>>  convexShape1 = shapeConvexDec.GetConvexShapeList(
			//		Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position,
			//			item.TriangleIndex.ToArray())),
			//		0.2);


			AABB testRegion = new AABB(
				new SharpEngineMathUtility.Vector3(-0.5,-0.3,-0.8),
				new SharpEngineMathUtility.Vector3(0.5, 0.7, 0.8));

			convexShape = shapeConvexDec.GetIntersectedShape(testRegion,
				testRegion,
				Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position,
						item.TriangleIndex.ToArray(),0)),
					0.2);

			List<Line> ll = OpenGLUtilities.BuildBoxLine(testRegion.Max, testRegion.Min);

			foreach (var item in ll)
			{
				GL.Color3(System.Drawing.Color.Red);
				OpenGLUtilities.DrawLine(item.a, item.b);
				GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
			}


			foreach (var shape in convexShape)
			{
				GL.Color3(GetRandomNumber(0.0, 1.0), GetRandomNumber(0.0, 1.0), GetRandomNumber(0.0, 1.0));

				IVertex[] vtx = Array.ConvertAll(shape.Vertex3Idx.ToArray(), x => new DefaultVertex() { Position = x.Vector3.Array });

				ConvexHull<IVertex, DefaultConvexFace<IVertex>> cHull = ConvexHull.Create(vtx);

				var convexHullShape = Array.ConvertAll(cHull.Faces.ToArray(), x => x.Vertices);

				var convert = Array.ConvertAll(convexHullShape, x => Array.ConvertAll(x, y => new SharpEngineMathUtility.Vector3(y.Position)));               

				OpenGLUtilities.GLDrawSolid(convert, new SharpEngineMathUtility.Vector3(0.0,0.0,0.0), false, false, false);

				GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
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


		private void displaySphere(List<NonConvexSphereDecomposition.NonConvexPoint> spherePoint)
		{
			for (int i = 0; i < spherePoint.Count; i++)
			{
				GL.PushMatrix();

				Matrix4 mView = Matrix4.CreateTranslation(
									Convert.ToSingle(spherePoint[i].IntersectionPoint.x),
									Convert.ToSingle(spherePoint[i].IntersectionPoint.y),
									Convert.ToSingle(spherePoint[i].IntersectionPoint.z));

				var dmviewData = new float[] {
						mView.M11, mView.M12, mView.M13, mView.M14,
						mView.M21, mView.M22, mView.M23, mView.M24,
						mView.M31, mView.M32, mView.M33, mView.M34,
						mView.M41, mView.M42, mView.M43, mView.M44
					};

				GL.MultMatrix(dmviewData);

				GL.Color4(1.0f, 0.0f, 0.0f, 1.0f);
				OpenGLUtilities.drawSolidCube(0.08f);
				GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);

				GL.PopMatrix();

			}
			
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

