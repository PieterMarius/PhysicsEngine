using System;
using System.Collections.Generic;
using System.Diagnostics;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using Utility;
using ShapeDefinition;
using MonoPhysicsEngine;
using CollisionEngine;
using LCPSolver;
using System.Linq;

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
		SimulationParameters simulationParameters;
		PhysicsEngine physicsEngine;

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

        //void TestNumeric()
        //{
        //    int nvalue = 10000000;
        //    stopwatch.Reset();
        //    stopwatch.Start();
        //    System.Numerics.Vector3[] testVector = new System.Numerics.Vector3[nvalue];

        //    //double[] vv = new double[] { 0.0, 0.0, 0.0, 0.0 };
        //    for (int i = 0; i < nvalue; i++)
        //        testVector[i] = new System.Numerics.Vector3(0, 0, 0);

        //    System.Numerics.Vector3 test;
        //    for (int i = 0; i < nvalue; i++)
        //        test = testVector[i] * 1.0f;

        //    stopwatch.Stop();
        //    Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

        //    stopwatch.Reset();
        //    stopwatch.Start();

        //    PhysicsEngineMathUtility.Vector3[] testVector1 = new PhysicsEngineMathUtility.Vector3[nvalue];

        //    for (int i = 0; i < nvalue; i++)
        //        testVector1[i] = new PhysicsEngineMathUtility.Vector3(0.0, 0.0, 0.0);

        //    PhysicsEngineMathUtility.Vector3 test1;
        //    for (int i = 0; i < nvalue; i++)
        //        test1 = testVector1[i] * 1.0;

        //    stopwatch.Stop();
        //    Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);


        //}

        public struct ShapePiece
        {
            public PhysicsEngineMathUtility.Vector3 IntersectionPoint; //Max => 8 index points
            public VertexAdjacency[] Triangle; 
        };
        
        List<PhysicsEngineMathUtility.Vector3> spherePoint = new List<PhysicsEngineMathUtility.Vector3>();
        void TestNonConvexDec()
        {
            //Utility.ObjImporter importer = new ObjImporter();
            //Utility.ObjImporter.meshStruct mesh = importer.ImportFile("chair.obj");

            IGeometry obj = LoadObject.GetObjectGeometry(null, "bunny.obj", 1.0f, ObjectGeometryType.NonConvexBody);
            //Geometry obj = LoadObject.GetObjectGeometry(null, "chair.obj", 1.0f);

            

            spherePoint = new List<PhysicsEngineMathUtility.Vector3>();

            //Suddivido il volume in parti
            int nSphere = 20;

            var test = obj.VertexPosition.Where(x => x.Adjacency.Count < 6).ToList();

            List<ShapePiece> basePoint = new List<ShapePiece>();


            //Provo a verificare di farlo per ogni lato del cubo
            double sphereGap = 2.1 / nSphere;

            //Direction 1
            PhysicsEngineMathUtility.Vector3 direction1 = new PhysicsEngineMathUtility.Vector3(0.0, 0.0, -1.0);
            PhysicsEngineMathUtility.Vector3 direction2 = new PhysicsEngineMathUtility.Vector3(0.0, 0.0, 1.0);
            PhysicsEngineMathUtility.Vector3 direction3 = new PhysicsEngineMathUtility.Vector3(0.0, -1.0, 0.0);
            PhysicsEngineMathUtility.Vector3 direction4 = new PhysicsEngineMathUtility.Vector3(0.0, 1.0, 0.0);
            PhysicsEngineMathUtility.Vector3 direction5 = new PhysicsEngineMathUtility.Vector3(-1.0, 0.0, 0.0);
            PhysicsEngineMathUtility.Vector3 direction6 = new PhysicsEngineMathUtility.Vector3(1.0, 0.0, 0.0);

            PhysicsEngineMathUtility.Vector3 startPoint1 = new PhysicsEngineMathUtility.Vector3(-1.05, -1.05, 0.0);
            PhysicsEngineMathUtility.Vector3 startPoint2 = new PhysicsEngineMathUtility.Vector3(-1.05, 0.0, -1.05);
            PhysicsEngineMathUtility.Vector3 startPoint3 = new PhysicsEngineMathUtility.Vector3(0.0, -1.05, -1.05);


            stopwatch.Reset();
            stopwatch.Start();
            //Parallel.For(0,
            //    nSphere,
            //    new ParallelOptions { MaxDegreeOfParallelism = 4 },
            //    i =>
            for (int i = 0; i < nSphere; i++)
            {
                for (int k = 0; k < nSphere; k++)
                {
                    PhysicsEngineMathUtility.Vector3 point1 = startPoint1 + new PhysicsEngineMathUtility.Vector3(i * sphereGap, k * sphereGap, 0.0);
                    PhysicsEngineMathUtility.Vector3 point2 = startPoint2 + new PhysicsEngineMathUtility.Vector3(i * sphereGap, 0.0, k * sphereGap);
                    PhysicsEngineMathUtility.Vector3 point3 = startPoint3 + new PhysicsEngineMathUtility.Vector3(0.0, i * sphereGap, k * sphereGap);



                    for (int j = 0; j < obj.Triangle.Length; j++)
                    {
                        PhysicsEngineMathUtility.Vector3 vertexA = obj.VertexPosition[obj.Triangle[j][0]].Vertex;
                        PhysicsEngineMathUtility.Vector3 vertexB = obj.VertexPosition[obj.Triangle[j][1]].Vertex;
                        PhysicsEngineMathUtility.Vector3 vertexC = obj.VertexPosition[obj.Triangle[j][2]].Vertex;


                        PhysicsEngineMathUtility.Vector3? intersection = PhysicsEngineMathUtility.GeometryUtilities.RayTriangleIntersection(
                                vertexA,
                                vertexB,
                                vertexC,
                                point1,
                                direction1,
                                true);

                        if (intersection.HasValue)
                        {
                            spherePoint.Add(intersection.Value);
                            basePoint.Add(new ShapePiece() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[obj.Triangle[j][0]], obj.VertexPosition[obj.Triangle[j][1]], obj.VertexPosition[obj.Triangle[j][2]] } });
                            continue;
                        }

                        intersection = PhysicsEngineMathUtility.GeometryUtilities.RayTriangleIntersection(
                            vertexA,
                            vertexB,
                            vertexC,
                            point2,
                            direction3,
                            true);

                        if (intersection.HasValue)
                        {
                            spherePoint.Add(intersection.Value);
                            basePoint.Add(new ShapePiece() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[obj.Triangle[j][0]], obj.VertexPosition[obj.Triangle[j][1]], obj.VertexPosition[obj.Triangle[j][2]] } });
                            continue;
                        }

                        intersection = PhysicsEngineMathUtility.GeometryUtilities.RayTriangleIntersection(
                            vertexA,
                            vertexB,
                            vertexC,
                            point3,
                            direction5,
                            true);

                        if (intersection.HasValue)
                        {
                            spherePoint.Add(intersection.Value);
                            basePoint.Add(new ShapePiece() { IntersectionPoint = intersection.Value, Triangle = new VertexAdjacency[3] { obj.VertexPosition[obj.Triangle[j][0]], obj.VertexPosition[obj.Triangle[j][1]], obj.VertexPosition[obj.Triangle[j][2]] } });
                            continue;
                        }
                                               
                    }


                }
            }

            double ray = sphereGap / 2.0;
            List<int>[] index = new List<int>[spherePoint.Count];
            int totalCount = 0;
            for (int i = 0; i < spherePoint.Count; i++)
            {
                PhysicsEngineMathUtility.Vector3 point = spherePoint[i];
                index[i] = new List<int>();
                for (int j = 0; j < obj.VertexPosition.Length; j++)
                {
                    if(PhysicsEngineMathUtility.Vector3.Length(point- obj.VertexPosition[j].Vertex) <= ray)
                        index[i].Add(j);
                }
                totalCount += index[i].Count;
            }

            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);
        }

        void initProgram()
		{
			try
			{
                //TestNumeric();
                //var env = new BuildEnvironment();
                //env.GetPhysicsEnvironment();

                //LoadObject loadObject = new LoadObject ("startJoint.xml");
                //LoadObject loadObject = new LoadObject ("configJoint.xml");
                //var loadObject = new LoadObject ("startConfig.xml");
                //var loadObject = new LoadObject ("carConfig.xml");
                //var loadObject = new LoadObject("testJointBridge.xml");
                //var loadObject = new LoadObject("compositeObjectConfig.xml");
                //var loadObject = new LoadObject("frictionTestConfig.xml");
               TestNonConvexDec();

                //            var loadObject = new LoadObject("softBodyConfig.xml");

                //            simulationObjects = loadObject.LoadSimulationObjects ();
                //simulationJoints = loadObject.LoadSimulationJoints (simulationObjects);

                displayList = LoadObject.GetOpenGLObjectList("bunny.obj", 1);

                //Carico le texture
                //textureID = loadObject.LoadTexture();
                redTexture = OpenGLUtilities.LoadTexture("red.bmp");

                //Set Collision Detection
                //collisionEngineParameters = new CollisionEngineParameters();

                //Set Solver
                //solverParameters = new SolverParameters();

                //Set Physics engine
                //simulationParameters = new SimulationParameters();

                //physicsEngine = new PhysicsEngine(
                //	simulationParameters,
                //	collisionEngineParameters,
                //	solverParameters);

                //physicsEngine.SetSolver(SolverType.ProjectedGaussSeidel);

                //for (int i = 0; i < simulationObjects.Count (); i++) 
                //{
                //	physicsEngine.AddObject (simulationObjects [i]);
                //}

                //for (int i = 0; i < simulationJoints.Count (); i++) 
                //{
                //	physicsEngine.AddJoint (simulationJoints [i]);
                //}

                pause = true;

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
                collisionPartitionedPoints = new List<List<CollisionPointStructure>>();

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

            //displayOrigin ();
            //displayContact ();
            //displayBaseContact();
            //displayJoint ();
            displaySphere();
            DisplayObject();

            //displayPartitionedContact();

            //for (int i = 0; i < physicsEngine.ObjectCount(); i++)
            //    displayVertex(i);

            //displayAABB();
            //displayVertex (0);
            //displayVertex (1);
            //displayVertex (2);

            //for (int i = 0; i < physicsEngine.ObjectCount(); i++)
            //    SetOpenGLObjectMatrixAndDisplayObject(i);

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
                    
					//physicsEngine.Simulate();
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



					pause = true;
                    //if (elapsedTime > 6.0)
                    //	Exit();
                    collPoint = physicsEngine.GetCollisionPointStrucureList();
                    collisionPartitionedPoints = physicsEngine.GetPartitionedCollisionPoints();

                    colorList = new List<List<double>>();
                    if (collisionPartitionedPoints != null)
                    {
                        for (int i = 0; i < collisionPartitionedPoints.Count; i++)
                        {
                            List<double> color = new List<double>();
                            color.Add(GetRandomNumber(0.0, 1.0));
                            color.Add(GetRandomNumber(0.0, 1.0));
                            color.Add(GetRandomNumber(0.0, 1.0));
                            colorList.Add(color);
                        }
                    }
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

        private void DisplayObject()
        {
            for (int i = 0; i < displayList.Length; i++)
            {
                for (int j = 0; j < displayList[i].Length; j++)
                {
                    GL.PushMatrix();
                    GL.Enable(EnableCap.Texture2D);

                    //GL.BindTexture(TextureTarget.Texture2D, textureID[id][i]);

                    GL.CallList(displayList[i][j]);
                    GL.Disable(EnableCap.Texture2D);

                    GL.PopMatrix();
                }
            }
        }

		private void SetOpenGLObjectMatrixAndDisplayObject(int id)
		{
			// TODO parte da modificare
			//Matrice da utilizzare come costante
			PhysicsEngineMathUtility.Matrix3x3 rotatioMatrix = physicsEngine.GetObject (id).RotationMatrix;
			PhysicsEngineMathUtility.Vector3 position = physicsEngine.GetObject (id).Position;
			ObjectType type = physicsEngine.GetObject (id).ObjectType;
            
            for (int i = 0; i < ShapeDefinition.Helper.GetGeometry(physicsEngine.GetObject(id)).Length; i++)
            {
                GL.PushMatrix();
                GL.Enable(EnableCap.Texture2D);

                PhysicsEngineMathUtility.Vector3 compoundPos = new PhysicsEngineMathUtility.Vector3();

                if (physicsEngine.GetObject(id) is ICompoundShape)
                    compoundPos = ((ICompoundShape)physicsEngine.GetObject(id)).StartCompoundPositionObjects[i];

                PhysicsEngineMathUtility.Vector3 positionMt = position + compoundPos-
                                                              physicsEngine.GetObject(id).StartPosition;

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
                GL.Translate(positionMt.x - position.x, positionMt.y - position.y, positionMt.z-position.z);
                

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
                            Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.x),
                            Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.y),
                            Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.z));

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
                            Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.x),
                            Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.y),
                            Convert.ToSingle(collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.z));

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
                                    Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointA.x),
                                    Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointA.y),
                                    Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointA.z));

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
                                    Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointB.x),
                                    Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointB.y),
                                    Convert.ToSingle(collPointStr[i].CollisionPointBase[n].CollisionPoints[j].CollisionPointB.z));

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
                        Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointA.x),
                        Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointA.y),
                        Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointA.z));

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
                        Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointB.x),
                        Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointB.y),
                        Convert.ToSingle(collPoint[i].CollisionPointBase[n].CollisionPoint.CollisionPointB.z));

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

			for (int i = 0; i < ShapeDefinition.Helper.GetGeometry(physicsEngine.GetObject (index)).Length; i++) 
			{
                for (int j = 0; j < ShapeDefinition.Helper.GetGeometry(physicsEngine.GetObject(index))[i].RelativePosition.Length; j++)
                {
                    PhysicsEngineMathUtility.Vector3 relativePosition = physicsEngine.GetObject(index).Position +
                                    (physicsEngine.GetObject(index).RotationMatrix * ShapeDefinition.Helper.GetGeometry(physicsEngine.GetObject(index))[i].RelativePosition[j]);

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
            IShape[] simObj = physicsEngine.GetSimulationObjects();
            for (int i = 0; i < simObj.Length; i++)
            {
                AABB joint = ShapeDefinition.Helper.GetGeometry(simObj[i])[0].AABBox;
                                
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


        private void displaySphere()
        {
            for (int i = 0; i < spherePoint.Count; i++)
            {
                GL.PushMatrix();

                Matrix4 mView = Matrix4.CreateTranslation(
                                    Convert.ToSingle(spherePoint[i].x),
                                    Convert.ToSingle(spherePoint[i].y),
                                    Convert.ToSingle(spherePoint[i].z));

                var dmviewData = new float[] {
                        mView.M11, mView.M12, mView.M13, mView.M14,
                        mView.M21, mView.M22, mView.M23, mView.M24,
                        mView.M31, mView.M32, mView.M33, mView.M34,
                        mView.M41, mView.M42, mView.M43, mView.M44
                    };

                GL.MultMatrix(dmviewData);

                GL.Color4(1.0f, 0.0f, 0.0f, 1.0f);
                OpenGLUtilities.drawSolidCube(0.01f);
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

