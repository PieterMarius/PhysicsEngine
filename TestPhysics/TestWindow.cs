﻿/******************************************************************************
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
using SharpEngineMathUtility;
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpPhysicsEngine.Wrapper;
using SharpPhysicsEngine.Wrapper.Joint;
using SharpPhysicsEngine.NonConvexDecomposition;
using MIConvexHull;

namespace TestPhysics
{
    public class TestWindow : GameWindow
    {
        public TestWindow() : base(32, 32, new GraphicsMode(32, 24, 0, 4))
        {
        }

        Stopwatch stopwatch = new Stopwatch();

        int[][] textureID;

        int[][] displayList;

        double elapsedTime = 0.0;
        long performaceValue = 0;

        List<CollisionPointStructure> collPoint;
        List<List<CollisionPointStructure>> collisionPartitionedPoints;

        ICollisionShape[] simulationObjects;
        ICollisionJoint[] simulationJoints;

        SharpEngine physicsEngine;

        List<List<double>> colorList = new List<List<double>>();

        bool pause = false;

        int redTexture;

        double[][] TerrainPositions;
        double[][][] TerrainTexture;
        double[][][] TerrainShapes;

        SharpEngineMathUtility.Vector3d? selectedPoint = null;

        #region Keyboard and mouse variables

        OpenTK.Input.MouseState current, previous;
        float xrot = 0.0f;
        float yrot = 0.0f;
        float xpos = 0.0f;
        float ypos = 0.0f;//-4.0f;
        float zpos = 0.0f;//-7.0f;

        #endregion

        void TestNumeric()
        {
            Console.WriteLine("Hardware accellerate " + System.Numerics.Vector.IsHardwareAccelerated);
            int nvalue = 3000000;
            stopwatch.Reset();
            

            System.Numerics.Vector<double>[] testVector = new System.Numerics.Vector<double>[nvalue];

            //double[] vv = new double[] { 0.0, 0.0, 0.0, 0.0 };
            for (int i = 0; i < nvalue; i++)
                testVector[i] = new System.Numerics.Vector<double>(new double[] { 2.0, 3.0, 5.0, 0.0 });

            stopwatch.Start();
            System.Numerics.Vector<double> test;
            double test1;
            for (int i = 0; i < nvalue; i++)
                test1 = System.Numerics.Vector.Dot(testVector[i], testVector[i]);

            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

            stopwatch.Reset();
            

            SharpEngineMathUtility.Vector3d[] testVector1 = new SharpEngineMathUtility.Vector3d[nvalue];

            for (int i = 0; i < nvalue; i++)
                testVector1[i] = new SharpEngineMathUtility.Vector3d(2.0, 3.0, 5.0);

            stopwatch.Start();
            //SharpEngineMathUtility.Vector3d test1;
            //double test2;
            for (int i = 0; i < nvalue; i++)
                test1 = testVector1[i].Dot(testVector1[i]);

            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

            

            //System.Numerics.Vector<double>[] testVector2 = new System.Numerics.Vector<double>[nvalue];

            for (int i = 0; i < nvalue; i++)
                testVector1[i] = new SharpEngineMathUtility.Vector3d(2.0, 3.0, 5.0);

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
            stopwatch.Reset();
            stopwatch.Start();

            test2 = SIMDUtils.SIMDArrayProductScalar(test3.ToArray(), 2.0);

            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);

            

            //System.Numerics.Vector<double>[] testVector2 = new System.Numerics.Vector<double>[nvalue];

            for (int i = 0; i < nvalue; i++)
                testVector1[i] = new SharpEngineMathUtility.Vector3d(2.0, 3.0, 5.0);

            //{ new SharpEngineMathUtility.Vector3(0.0, 0.0, 0.0) };


            //System.Numerics.Vector<double> tt = new System.Numerics.Vector<double>(new SharpEngineMathUtility.Vector3(1.0, 0.0, 0.0));
            //List<double> test4 = new List<double>();
            ////double[] test4 = new double[nvalue * 3];

            //for (int i = 0; i < nvalue; i++)
            //{
            //    // int index = (i * 3);
            //    test4.AddRange(testVector1[i].ToList);
            //    //test4[index] = testVector1[i].x;
            //    //test4[index +1] = testVector1[i].y;
            //    //test4[index +2] = testVector1[i].z;

            //    //test3.AddRange(testVector1[i].Array);
            //    //test3.Add(0);


            //}

            stopwatch.Reset();
            stopwatch.Start();

            double[] out4 = new double[test3.Count];
            for (int i = 0; i < test3.Count; i++)
            {
                // int index = (i * 3);
                out4[i] = test3[i] * 2.0;
                //test4[index] = testVector1[i].x;
                //test4[index +1] = testVector1[i].y;
                //test4[index +2] = testVector1[i].z;

                //test3.AddRange(testVector1[i].Array);
                //test3.Add(0);


            }


            stopwatch.Stop();
            Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);
            Console.ReadLine();


        }

        NonConvexSphereDecomposition testConvexDecomp = new NonConvexSphereDecomposition();

        List<Line> octTreeLine = new List<Line>();
        void initProgram()
        {
            //TestNumeric();
            try
            {
                //testConvexDecomp.Decompose(0.08);

                //TestNumeric();

                //env.GetPhysicsEnvironment();

                //LoadEngineByXml();

                LoadEngineByBuilder();


            }
            catch (Exception e)
            {
                throw new Exception(e.StackTrace);
            }
        }

        private void LoadEngineByBuilder()
        {
            var env = new BuildEnvironment2();

            physicsEngine = env.GetPhysicsEnvironment();
            displayList = env.GetOpenGLEnvironment();
            textureID = env.LoadTexture();

            redTexture = OpenGLUtilities.LoadTexture("red.bmp");

            //TerrainMesh terrain = new TerrainMesh();
            //TerrainPositions = terrain.GetPositions();
            //TerrainTexture = terrain.GetTextureCoordMatrix();
            //TerrainShapes = terrain.GetConvexShapeList();
            //InitTerrain();

            //string softObject = "sph.obj";

            //var objects3 = BuildSoftBody(softObject, 1, new SharpEngineMathUtility.Vector3(0.0, 8.0, 1.5));
            //objects3.SetStaticFrictionCoeff(0.4);
            //objects3.SetDynamicFrictionCoeff(0.3);
            //objects3.SetRestitutionCoeff(1.0);
            //objects3.SetErrorReductionParam(1.0);

            //physicsEngine.AddShape(objects3);

            pause = true;

            collPoint = new List<CollisionPointStructure>();
            collisionPartitionedPoints = new List<List<CollisionPointStructure>>();
        }

        private void LoadEngineByXml()
        {
            //LoadObject loadObject = new LoadObject("startJoint.xml");
            //LoadObject loadObject = new LoadObject("configJoint.xml");
            //var loadObject = new LoadObject("startConfig.xml");
            //var loadObject = new LoadObject("carConfig.xml");
            //var loadObject = new LoadObject("testJointBridge.xml");
            //var loadObject = new LoadObject("compositeObjectConfig.xml");
            var loadObject = new LoadObject("frictionTestConfig.xml");

            //var loadObject = new LoadObject("softBodyConfig.xml");


            simulationObjects = loadObject.LoadSimulationObjects();
            simulationJoints = loadObject.LoadSimulationJoints(simulationObjects);

            displayList = loadObject.GetOpenGLObjectList();

            ////Carico le texture
            textureID = loadObject.LoadTexture();
            redTexture = OpenGLUtilities.LoadTexture("red.bmp");

            physicsEngine = new SharpEngine();

            physicsEngine.SetSolverType(SolverType.NonLinearConjugateGradient);
            physicsEngine.SolverParameters.SetSolverMaxIteration(30);

            for (int i = 0; i < simulationObjects.Count(); i++)
            {
                physicsEngine.AddShape(simulationObjects[i]);
            }

            for (int i = 0; i < simulationJoints.Count(); i++)
            {
                physicsEngine.AddJoint(simulationJoints[i]);
            }

            //string softObject = "sph.obj";

            //var objects3 = BuildSoftBody(softObject, 1, new SharpEngineMathUtility.Vector3(0.0, 8.0, 1.5));
            //objects3.SetStaticFrictionCoeff(0.4);
            //objects3.SetDynamicFrictionCoeff(0.3);
            //objects3.SetRestitutionCoeff(1.0);
            //objects3.SetErrorReductionParam(1.0);

            //physicsEngine.AddShape(objects3);

            pause = true;

            collPoint = new List<CollisionPointStructure>();
            collisionPartitionedPoints = new List<List<CollisionPointStructure>>();

        }

        private SoftShape BuildSoftBody(
            string fileName,
            double scale,
            SharpEngineMathUtility.Vector3d position)
        {
            GenericUtility.ObjProperties prop = GenericUtility.GetImportedObjectProperties(fileName, scale);

            //RotateObj(ref prop, new Vector3(0.0, 0.0, 1.0), -Math.PI / 4.5);

            return new SoftShape(
                prop.triangleIndex,
                prop.vertexPoint,
                position,
                4.0,
                0.2,
                0.9,
                0.01,
                0.1,
                0.1);
        }


        #region OpenGL Windows Settings

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            initProgram();

            Title = "Physics Window";

            //OpenGL Windows properties settings
            GL.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            GL.Enable(EnableCap.DepthTest);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.One);
            GL.DepthFunc(DepthFunction.Lequal);

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadIdentity();
           
            MoveCamera();

            cameraDirection = GetCameraDirection();
            

            //displayTerrain(TerrainPositions, TerrainTexture, 256, 256);
            //displayOrigin ();
            displayContact();
            //DisplayHierarchicalTree();
            //DisplayHierarchicalIntersection();
            //DisplayConcaveShape(1);
            //DisplayConcaveShape(2);
            //displayBaseContact();
            displayJoint();
            displayHingeJoint();

            DrawCameraDirection(cameraDirection);
            VisualizeSelectedPoint();
            //displaySoftJoint();
            //displaySphere(testConvexDecomp.basePoint);
            //DisplayObject();

            //displayPartitionedContact();

            //for (int i = 0; i < physicsEngine.ObjectCount(); i++)
            //    displayVertex(i);                 

            //displayAABB();
            //displayVertex(0);
            //displayVertex(1);
            //displayVertex(73);


            for (int i = 0; i < physicsEngine.ShapesCount(); i++)
                SetOpenGLObjectMatrixAndDisplayObject(i);

            //displayOctree();
            //displayConvexDecomposition();
            //displaySoftJoint();

            GL.Flush();
            SwapBuffers();


        }
        double timeStep = 0.007;
        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.Escape])
                Exit();

            GL.Viewport(0, 0, Width, Height);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            OpenGLUtilities.GluPerspective(
                60,
                Convert.ToDouble(Width) / Convert.ToDouble(Height),
                1.0,
                100.0);

            GL.MatrixMode(MatrixMode.Modelview);

            //Physics
            UpdateMouse();
            UpdateKeyboard();

            try {

                if (!pause)
                {
                    stopwatch.Reset();
                    stopwatch.Start();

                    collPoint.Clear();

                    physicsEngine.Simulate(timeStep);

                    stopwatch.Stop();

                    elapsedTime += timeStep;
                    performaceValue += stopwatch.ElapsedMilliseconds;

                    Console.WriteLine("Engine Elapsed={0}", stopwatch.ElapsedMilliseconds);
                    Console.WriteLine("ElapsedTime " + elapsedTime + " performanceValue " + performaceValue);
                    Console.WriteLine("Partial " + stopwatch.ElapsedMilliseconds);
                    Console.WriteLine();


                    collPoint = physicsEngine.GetCollisionPointStrucureList();
                    //if (collPoint.Count > 0)
                    //    pause = true;


                    //pause = true;
                    //if (elapsedTime > 6.0)
                    //	Exit();

                    //if (collPoint.Count > 0)
                    //    Console.WriteLine();
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
                throw new Exception("Physics engine error. " + ex.StackTrace);
            }


        }

        #region TEST

        protected void setOrthographicProjection()
        {
            GL.MatrixMode(MatrixMode.Projection);

            GL.PushMatrix();

            GL.LoadIdentity();

        }

        #endregion

        #endregion

        SharpEngineMathUtility.Vector3d cameraDirection = new SharpEngineMathUtility.Vector3d();

        protected void MoveCamera()
        {
            GL.Rotate(xrot, 1.0, 0.0, 0.0);  //rotate 
            GL.Rotate(yrot, 0.0, 1.0, 0.0);  //rotate 
            GL.Translate(xpos, ypos, zpos); //translate the screen
            
        }

        #region Mouse Input

        public void UpdateMouse()
        {
            current = OpenTK.Input.Mouse.GetCursorState();

            if (current != previous)
            {
                int xdelta = current.X - previous.X;
                int ydelta = current.Y - previous.Y;
                xrot += Convert.ToSingle(ydelta);
                yrot += Convert.ToSingle(xdelta);
                if (xrot > 90.0f) xrot = 90.0f;
                if (yrot < -90.0f) yrot = -90.0f;
            }
            previous = current;


            if(OpenTK.Input.Mouse.GetState()[OpenTK.Input.MouseButton.Left])
            {
                var origin = new SharpEngineMathUtility.Vector3d(-xpos, -ypos, -zpos);
                var direction = cameraDirection;
                var ID = physicsEngine.RayCastShapeID(origin, direction);

                selectedObjIndex = -1;

                if (ID.HasValue)
                {
                    selectedObjIndex = ID.Value;

                }
            }

            if (OpenTK.Input.Mouse.GetState()[OpenTK.Input.MouseButton.Right])
            {
                var origin = new SharpEngineMathUtility.Vector3d(-xpos, -ypos, -zpos);
                var direction = cameraDirection;
                selectedPoint = physicsEngine.RayCastShape(origin, direction);
            }
        }

        #endregion

        #region Keyboard input

        int selectedObjIndex = -1;

        protected void UpdateKeyboard()
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
                yrotrad = (yrot / 180.0f * 3.141592654f);
                xrotrad = (xrot / 180.0f * 3.141592654f);
                xpos -= Convert.ToSingle(Math.Sin(yrotrad)) * 0.1f;
                zpos += Convert.ToSingle(Math.Cos(yrotrad)) * 0.1f;
                ypos += Convert.ToSingle(Math.Sin(xrotrad)) * 0.1f;
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.D])
            {
                float yrotrad;
                yrotrad = (yrot / 180.0f * 3.141592654f);
                xpos += Convert.ToSingle(Math.Cos(yrotrad)) * 0.1f;
                zpos += Convert.ToSingle(Math.Sin(yrotrad)) * 0.1f;
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.A])
            {
                float yrotrad;
                yrotrad = (yrot / 180.0f * 3.141592654f);
                xpos -= Convert.ToSingle(Math.Cos(yrotrad)) * 0.1f;
                zpos -= Convert.ToSingle(Math.Sin(yrotrad)) * 0.1f;
            }

            // pause
            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.P])
            {
                selectedObjIndex = -1;
                if (pause)
                {
                    pause = false;
                }
                else
                {
                    pause = true;
                }
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.R])
            {
                physicsEngine.Dispose();
                OnLoad(null);
            }

            //if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.F])
            //{
            //    pause = true;
            //    selectedObjIndex = Convert.ToInt32(Console.ReadLine());
            //}

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.KeypadPlus])
            {

                SoftShape softShape = physicsEngine.GetShape(3) as SoftShape;
                softShape.AddToConstraintsRestoreCoefficient(0.05);
                Console.WriteLine("Rest coeff " + softShape.GetShapeErrorReductionParams()[0]);

            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.KeypadMinus])
            {

                SoftShape softShape = physicsEngine.GetShape(3) as SoftShape;
                softShape.AddToConstraintsRestoreCoefficient(-0.05);
                Console.WriteLine("Rest coeff " + softShape.GetShapeErrorReductionParams()[0]);

            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.PageUp])
            {

                //SoftShape softShape = physicsEngine.GetShape(3) as SoftShape;
                //softShape.AddToConstraintsSpringCoefficient(0.05);

            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.PageDown])
            {

                //SoftShape softShape = physicsEngine.GetShape(3) as SoftShape;
                //softShape.AddToConstraintsSpringCoefficient(-0.05);

            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.Up])
            {
                pause = true;

                ((Hinge2Joint)physicsEngine.GetJoints(0)).AddTorqueShapeB(0.0, 5);
                ((Hinge2Joint)physicsEngine.GetJoints(1)).AddTorqueShapeB(0.0, 5);
                physicsEngine.Simulate(timeStep);
                pause = false;
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.Down])
            {
                pause = true;

                ((Hinge2Joint)physicsEngine.GetJoints(0)).AddTorqueShapeB(0.0, -5);
                ((Hinge2Joint)physicsEngine.GetJoints(1)).AddTorqueShapeB(0.0, -5);
                physicsEngine.Simulate(timeStep);
                pause = false;
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.Left])
            {
                pause = true;

                ((Hinge2Joint)physicsEngine.GetJoints(1)).SetAxis1AngularLimit(-0.78539816339, 0.78539816339);
                ((Hinge2Joint)physicsEngine.GetJoints(3)).SetAxis1AngularLimit(-0.78539816339, 0.78539816339);
                ((Hinge2Joint)physicsEngine.GetJoints(1)).AddTorqueShapeB(1.5, 0.0);
                //((Hinge2Joint)physicsEngine.GetJoints(3)).AddTorqueShapeB(1.5,0.0);

                physicsEngine.Simulate(timeStep);
                var angle = ((Hinge2Joint)physicsEngine.GetJoints(1)).GetAxis1Angle();

                ((Hinge2Joint)physicsEngine.GetJoints(1)).SetAxis1AngularLimit(angle, angle);
                ((Hinge2Joint)physicsEngine.GetJoints(3)).SetAxis1AngularLimit(angle, angle);

                pause = false;
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.Right])
            {
                pause = true;

                ((Hinge2Joint)physicsEngine.GetJoints(1)).SetAxis1AngularLimit(-0.78539816339, 0.78539816339);
                ((Hinge2Joint)physicsEngine.GetJoints(3)).SetAxis1AngularLimit(-0.78539816339, 0.78539816339);
                ((Hinge2Joint)physicsEngine.GetJoints(1)).AddTorqueShapeB(-1.5, 0.0);
                //((Hinge2Joint)physicsEngine.GetJoints(3)).AddTorqueShapeB(-1.5, 0.0);

                physicsEngine.Simulate(timeStep);
                var angle = ((Hinge2Joint)physicsEngine.GetJoints(1)).GetAxis1Angle();

                ((Hinge2Joint)physicsEngine.GetJoints(1)).SetAxis1AngularLimit(angle, angle);
                ((Hinge2Joint)physicsEngine.GetJoints(3)).SetAxis1AngularLimit(angle, angle);

                pause = false;
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.N])
            {
                pause = true;

                physicsEngine.GetShape(1).SetRotationStatus(new SharpEngineMathUtility.Quaternion(new SharpEngineMathUtility.Vector3d(0.0, 0.0, 1.0), Math.PI / 2.0));
                pause = false;
            }

            if (OpenTK.Input.Keyboard.GetState()[OpenTK.Input.Key.F])
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

        #region Private Methods

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

        private SharpEngineMathUtility.Vector3d GetCameraDirection()
        {
            Matrix4 mdl;

            GL.GetFloat(GetPName.ModelviewMatrix, out mdl);
            
            return new SharpEngineMathUtility.Vector3d(
                -mdl.M13,
                -mdl.M23,
                -mdl.M33).Normalize();
        }

        private void DrawCameraDirection(SharpEngineMathUtility.Vector3d cameraDirection)
        {
            var origin = new SharpEngineMathUtility.Vector3d(-xpos, -ypos, -zpos);
                        
            GL.Color3(0.0, 1.0f, 0.0);
            GL.PointSize(3.0f);
            GL.Begin(PrimitiveType.Points);
            var tt = origin + cameraDirection * 1.1;
            GL.Vertex3(tt.x, tt.y, tt.z);
            GL.End();
            GL.PointSize(1.0f);
            GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
        }

        private void VisualizeSelectedPoint()
        {
            if (selectedPoint.HasValue)
            {
                GL.Color3(0.0, 0.0f, 1.0f);
                GL.PointSize(10.0f);
                GL.Begin(PrimitiveType.Points);
                GL.Vertex3(selectedPoint.Value.x, selectedPoint.Value.y, selectedPoint.Value.z);
                GL.End();
                GL.PointSize(1.0f);
                GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
            }
        }

        private void SetOpenGLObjectMatrixAndDisplayObject(int id)
        {
            // TODO parte da modificare
            //Matrice da utilizzare come costante
            ICollisionShape shape = physicsEngine.GetShapes()[id];

            if (shape is SoftShape softShape)
                DisplaySoftPoint(softShape);
            else
            {
                Matrix3x3 rotatioMatrix = shape.RotationMatrix;
                SharpEngineMathUtility.Vector3d position = shape.Position;

                ObjectType type = shape.ObjectType;

                for (int i = 0; i < shape.GetGeometryCount(); i++)
                {
                    GL.PushMatrix();
                    GL.Enable(EnableCap.Texture2D);

                    SharpEngineMathUtility.Vector3d compoundPos = new SharpEngineMathUtility.Vector3d();

                    if (shape is CompoundRigidShape)
                        compoundPos = ((CompoundRigidShape)shape).StartCompoundPositionObjects[i];

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
                    //GL.Translate(-testTranslate.x, -testTranslate.y, -testTranslate.z);
                    //Ruoto
                    GL.MultMatrix(dmviewData);
                    //Traslo nella posizione desiderata
                    SharpEngineMathUtility.Vector3d positionMt = shape.GetCenterOfMassShiftValue(i);
                    //SharpEngineMathUtility.Vector3 positionMt = compoundPos;

                    //TODO Verificare che la rotazione sia corretta
                    GL.Translate(positionMt.x , positionMt.y , positionMt.z);


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

        private void DisplaySoftPoint(SoftShape softShape)
        {
            //GL.BindTexture(TextureTarget.Texture2D, textureID[3][0]);

            //GL.CallList(displayList[3][0]);
            //GL.Disable(EnableCap.Texture2D);

            //GL.PopMatrix();
            int i = 0;
            foreach (var item in softShape.GetVertices())
            {
                SharpEngineMathUtility.Vector3d relativePosition = item;
                
                if (softShape.GetShapePointsCount() - i <= 4)
                {
                    GL.Color3(1.0f, 0.0, 0.0);
                    GL.PointSize(3.0f);
                    GL.Begin(PrimitiveType.Points);
                    GL.Vertex3(relativePosition.x, relativePosition.y, relativePosition.z);
                    GL.End();
                    GL.PointSize(1.0f);
                    GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
                }
                else
                {
                    GL.PointSize(3.0f);
                    GL.Begin(PrimitiveType.Points);
                    GL.Vertex3(relativePosition.x, relativePosition.y, relativePosition.z);
                    GL.End();
                    GL.PointSize(1.0f);
                }
                                                               
                i++;
            }
        }

        private void displayContact()
        {
            //Console.WriteLine("NContact " + collPoint.Count);
            for (int i = 0; i < collPoint.Count; i++)
            {
                for (int h = 0; h < collPoint[i].CollisionPointBase.Length; h++)
                {
                    for (int j = 0; j < collPoint[i].CollisionPointBase[h].CollisionPoints.Length; j++)
                    {
                        GL.Color3(1.0f, 0.0, 0.0);
                        GL.PointSize(3.0f);
                        GL.Begin(PrimitiveType.Points);
                        GL.Vertex3(
                            collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.Vertex.x,
                            collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.Vertex.y,
                            collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointA.Vertex.z);
                        GL.Vertex3(
                            collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.Vertex.x,
                            collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.Vertex.y,
                            collPoint[i].CollisionPointBase[h].CollisionPoints[j].CollisionPointB.Vertex.z);

                        GL.End();
                        GL.PointSize(1.0f);
                        GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);

                        
                    }
                }
            }
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
            var shape = physicsEngine.GetShape(index);
            var vertices = shape.GetVertices();
            for (int i = 0; i < vertices.Length; i++)
            {
                var relativePosition = vertices[i];

                GL.Color3(1.0f, 0.0, 0.0);
                GL.PointSize(3.0f);
                GL.Begin(PrimitiveType.Points);
                GL.Vertex3(relativePosition.x, relativePosition.y, relativePosition.z);
                GL.End();
                GL.PointSize(1.0f);
                GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
            }
        }

        private void displayJoint()
		{
			for (int i = 0; i < physicsEngine.JointsCount(); i++) 
			{
                ICollisionJoint joint = physicsEngine.GetJoints(i);
                GL.Color3(1.0f, 0.0, 0.0);
                GL.PointSize(3.0f);
                GL.Begin(PrimitiveType.Points);
                var tt = joint.GetAnchorPosition();
                GL.Vertex3(tt.x, tt.y, tt.z);
                GL.End();
                GL.PointSize(1.0f);
                GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
			}
		}

        private void displayHingeJoint()
        {
            for (int i = 0; i < physicsEngine.JointsCount(); i++)
            {
                ICollisionJoint joint = physicsEngine.GetJoints(i);

                if (joint is Hinge2Joint)
                {

                    GL.Color3(1.0f, 0.0, 0.0);
                    var hingejoint = (Hinge2Joint)joint;

                    var hingeAxis = hingejoint.GetHingeAxis();
                    var hingeAxisPoint = joint.GetAnchorPosition() + hingeAxis;
                    OpenGLUtilities.DrawLine(joint.GetAnchorPosition(), hingeAxisPoint, 1.0f);

                    var rotationAxis = hingejoint.GetRotationAxis();
                    var rotationAxisPoint = joint.GetAnchorPosition() + rotationAxis;
                    OpenGLUtilities.DrawLine(joint.GetAnchorPosition(), rotationAxisPoint, 1.0f);

                    GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
                }

            }
        }

        private void displaySoftJoint()
        {
            SoftShape softShape = physicsEngine.GetShape(3) as SoftShape;

            foreach (var item in softShape.GetShapeConstraintsPosition())
            {
                GL.Color3(1.0f, 0.0, 0.0);
                GL.PointSize(3.0f);
                GL.Begin(PrimitiveType.Points);
                GL.Vertex3(item.x, item.y, item.z);
                GL.End();
                GL.PointSize(1.0f);
                GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
            }
        }

		private void displayAABB()
		{
			ICollisionShape[] simObj = physicsEngine.GetShapes();
			for (int i = 0; i < simObj.Length; i++)
			{
				SharpEngineMathUtility.Vector3d min = simObj[i].GetMinAABB();
                SharpEngineMathUtility.Vector3d max = simObj[i].GetMaxAABB();

                GL.PushMatrix();

				Matrix4 mView = Matrix4.CreateTranslation(
									Convert.ToSingle(max[0]),
									Convert.ToSingle(max[1]),
									Convert.ToSingle(max[2]));

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
									Convert.ToSingle(min[0]),
									Convert.ToSingle(min[1]),
									Convert.ToSingle(min[2]));

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

        private void DisplayHierarchicalTree()
        {
            var aabb = physicsEngine.GetHierarchicalTreeAABB();
            //var aabb = physicsEngine.GetShapesAABB();

            for (int i = 0; i < aabb.Count; i++)
            {
                SharpEngineMathUtility.Vector3d b1 = aabb[i].Item1;
                SharpEngineMathUtility.Vector3d b2 = aabb[i].Item2;

                SharpEngineMathUtility.Vector3d b3 = new SharpEngineMathUtility.Vector3d(b1.x, b1.y, b2.z);
                SharpEngineMathUtility.Vector3d b4 = new SharpEngineMathUtility.Vector3d(b1.x, b2.y, b1.z);
                SharpEngineMathUtility.Vector3d b5 = new SharpEngineMathUtility.Vector3d(b2.x, b1.y, b1.z);
                SharpEngineMathUtility.Vector3d b6 = new SharpEngineMathUtility.Vector3d(b1.x, b2.y, b2.z);
                SharpEngineMathUtility.Vector3d b7 = new SharpEngineMathUtility.Vector3d(b2.x, b1.y, b2.z);
                SharpEngineMathUtility.Vector3d b8 = new SharpEngineMathUtility.Vector3d(b2.x, b2.y, b1.z);

                OpenGLUtilities.DrawLine(b6, b2, 1.0f);
                OpenGLUtilities.DrawLine(b2, b8, 1.0f);
                OpenGLUtilities.DrawLine(b8, b4, 1.0f);
                OpenGLUtilities.DrawLine(b4, b6, 1.0f);

                OpenGLUtilities.DrawLine(b3, b7, 1.0f);
                OpenGLUtilities.DrawLine(b7, b5, 1.0f);
                OpenGLUtilities.DrawLine(b5, b1, 1.0f);
                OpenGLUtilities.DrawLine(b1, b3, 1.0f);

                OpenGLUtilities.DrawLine(b6, b3, 1.0f);
                OpenGLUtilities.DrawLine(b2, b7, 1.0f);
                OpenGLUtilities.DrawLine(b8, b5, 1.0f);
                OpenGLUtilities.DrawLine(b4, b1, 1.0f);
            }
        }

        private void DisplayHierarchicalIntersection()
        {
            return;
            //var aabb = physicsEngine.GetHierarchicalIntersection();
            ////var aabb = physicsEngine.GetShapesAABB();

            //for (int i = 0; i < aabb.Count; i++)
            //{
            //    SharpEngineMathUtility.Vector3d b1 = aabb[i].Item1;
            //    SharpEngineMathUtility.Vector3d b2 = aabb[i].Item2;

            //    SharpEngineMathUtility.Vector3d b3 = new SharpEngineMathUtility.Vector3d(b1.x, b1.y, b2.z);
            //    SharpEngineMathUtility.Vector3d b4 = new SharpEngineMathUtility.Vector3d(b1.x, b2.y, b1.z);
            //    SharpEngineMathUtility.Vector3d b5 = new SharpEngineMathUtility.Vector3d(b2.x, b1.y, b1.z);
            //    SharpEngineMathUtility.Vector3d b6 = new SharpEngineMathUtility.Vector3d(b1.x, b2.y, b2.z);
            //    SharpEngineMathUtility.Vector3d b7 = new SharpEngineMathUtility.Vector3d(b2.x, b1.y, b2.z);
            //    SharpEngineMathUtility.Vector3d b8 = new SharpEngineMathUtility.Vector3d(b2.x, b2.y, b1.z);

            //    GL.Color4(1.0f, 0.0f, 0.0f, 1.0f);

            //    OpenGLUtilities.DrawLine(b6, b2);
            //    OpenGLUtilities.DrawLine(b2, b8);
            //    OpenGLUtilities.DrawLine(b8, b4);
            //    OpenGLUtilities.DrawLine(b4, b6);

            //    OpenGLUtilities.DrawLine(b3, b7);
            //    OpenGLUtilities.DrawLine(b7, b5);
            //    OpenGLUtilities.DrawLine(b5, b1);
            //    OpenGLUtilities.DrawLine(b1, b3);

            //    OpenGLUtilities.DrawLine(b6, b3);
            //    OpenGLUtilities.DrawLine(b2, b7);
            //    OpenGLUtilities.DrawLine(b8, b5);
            //    OpenGLUtilities.DrawLine(b4, b1);

            //    GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
            //}
        }

        private void displayOctree()
		{
			foreach (var item in octTreeLine)
				OpenGLUtilities.DrawLine(item.a, item.b, 1.0f);
		}

        private void DisplayConcaveShape(int index)
        {
            ConcaveShape shape = physicsEngine.GetShape(index) as ConcaveShape;

            var convexShapeList = shape.GetConvexShapeList();

            for (int i = 0; i < convexShapeList.Length; i++)
            {
                IVertex[] vtx = Array.ConvertAll(convexShapeList[i], x => new DefaultVertex() { Position = x });

                ConvexHull<IVertex, DefaultConvexFace<IVertex>> cHull = ConvexHull.Create(vtx);

                var convexHullShape = Array.ConvertAll(cHull.Faces.ToArray(), x => x.Vertices);

                var convert = Array.ConvertAll(convexHullShape, x => Array.ConvertAll(x, y => new SharpEngineMathUtility.Vector3d(y.Position)));

                GL.Color3(OpenGLUtilities.GetRandomNumber(0.0, 1.0), OpenGLUtilities.GetRandomNumber(0.0, 1.0), OpenGLUtilities.GetRandomNumber(0.0, 1.0));

                OpenGLUtilities.GLDrawSolid(convert, new SharpEngineMathUtility.Vector3d(0.0, 0.0, 0.0), false, false, false);

                GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
            }
        }

		private void displayConvexDecomposition()
		{
			SoftShape softShape = physicsEngine.GetShape(1) as SoftShape;

            //region.Min = region.Min + new SharpEngineMathUtility.Vector3(-1.0, -1.0, -1.0);
            //region.Max = region.Max + new SharpEngineMathUtility.Vector3(1.0, 1.0, 1.0);

            //List<List<Vertex3Index>>  convexShape1 = shapeConvexDec.GetConvexShapeList(
            //		Array.ConvertAll(softShape.ShapePoints, item => new Vertex3Index(item.Position,
            //			item.TriangleIndex.ToArray())),
            //		0.2);

            //AABB region = softShape.AABBox;
                                   
			//AABB testRegion = new AABB(
			//	new SharpEngineMathUtility.Vector3(-0.5,-0.3,-0.8),
			//	new SharpEngineMathUtility.Vector3(0.5, 0.7, 0.8));

			var convexShape = ConvexDecompositionEngine.GetConvexShapeList(softShape);

			List<Line> ll = OpenGLUtilities.BuildBoxLine(softShape.GetMaxAABB(), softShape.GetMinAABB());

			foreach (var item in ll)
			{
				GL.Color3(System.Drawing.Color.Red);
				OpenGLUtilities.DrawLine(item.a, item.b, 1.0f);
				GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
			}

            int i = 0;
			foreach (var shape in convexShape)
			{
				GL.Color3(OpenGLUtilities.GetRandomNumber(0.0, 1.0), OpenGLUtilities.GetRandomNumber(0.0, 1.0), OpenGLUtilities.GetRandomNumber(0.0, 1.0));

				IVertex[] vtx = Array.ConvertAll(shape, x => new DefaultVertex() { Position = x });

                try
                {
                    ConvexHull<IVertex, DefaultConvexFace<IVertex>> cHull = ConvexHull.Create(vtx);

                    var convexHullShape = Array.ConvertAll(cHull.Faces.ToArray(), x => x.Vertices);

                    var convert = Array.ConvertAll(convexHullShape, x => Array.ConvertAll(x, y => new SharpEngineMathUtility.Vector3d(y.Position)));

                    OpenGLUtilities.GLDrawSolid(convert, new SharpEngineMathUtility.Vector3d(0.0, 0.0, 0.0), false, false, false);
                }
                catch (Exception ex)
                {
                    throw new Exception(ex.Message);
                }



                GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
                i++;
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

        IVertex[][][] ConvexHullShapes;
        SharpEngineMathUtility.Vector3d[][][] ConvertConvexHull;

        private void InitTerrain()
        {
            ConvexHullShapes = new IVertex[TerrainShapes.Length][][];
            ConvertConvexHull = new SharpEngineMathUtility.Vector3d[TerrainShapes.Length][][];

            for (int i = 0; i < TerrainShapes.Length; i++)
            {

           
                IVertex[] vtx = Array.ConvertAll(TerrainShapes[i], x => new DefaultVertex() { Position = x });

                if (vtx.Length > 5)
                {

                    ConvexHull<IVertex, DefaultConvexFace<IVertex>> cHull = ConvexHull.Create(vtx);

                    ConvexHullShapes[i] = Array.ConvertAll(cHull.Faces.ToArray(), x => x.Vertices);

                    ConvertConvexHull[i] = Array.ConvertAll(ConvexHullShapes[i], x => Array.ConvertAll(x, y => new SharpEngineMathUtility.Vector3d(y.Position)));

                }
            }
        }

        private void displayTerrain(double[][] spherePoint,double[][][] textureCoord, int height, int width)
        {
            //GL.Enable(EnableCap.Texture2D);

            //GL.BindTexture(TextureTarget.Texture2D, textureID[0][0]);

            //OpenGLUtilities.DrawTerrain(spherePoint, textureCoord, height, width);

            //GL.Disable(EnableCap.Texture2D);


            //GL.Enable(EnableCap.Texture2D);

            //GL.BindTexture(TextureTarget.Texture2D, textureID[0][0]);

            foreach (var shape in ConvertConvexHull)
            {
                if (shape != null)
                {
                    GL.Color3(OpenGLUtilities.GetRandomNumber(0.0, 1.0), OpenGLUtilities.GetRandomNumber(0.0, 1.0), OpenGLUtilities.GetRandomNumber(0.0, 1.0));
                    //GL.Color3(0.0, 1.0, 0.0);


                    //var convert = Array.ConvertAll(shape, x => Array.ConvertAll(x, y => new SharpEngineMathUtility.Vector3(y.Position)));

                    OpenGLUtilities.GLDrawSolid(shape, new SharpEngineMathUtility.Vector3d(0.0, 0.0, 0.0), false, false, false);

                    GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
                }
            }

            //GL.Disable(EnableCap.Texture2D);

            //int index = 0;
            //for (int row = 0; row < height; row++)
            //{
            //    for (int col = 0; col < width; col++)
            //    {
            //        GL.PushMatrix();

            //        Matrix4 mView = Matrix4.CreateTranslation(
            //                            Convert.ToSingle(spherePoint[index]),
            //                            Convert.ToSingle(spherePoint[index+1]),
            //                            Convert.ToSingle(spherePoint[index+2]));

            //        var dmviewData = new float[] {
            //            mView.M11, mView.M12, mView.M13, mView.M14,
            //            mView.M21, mView.M22, mView.M23, mView.M24,
            //            mView.M31, mView.M32, mView.M33, mView.M34,
            //            mView.M41, mView.M42, mView.M43, mView.M44
            //        };

            //        GL.MultMatrix(dmviewData);

            //        GL.Color4(1.0f, 0.0f, 0.0f, 1.0f);
            //        OpenGLUtilities.drawPoint(1.0f);
            //        GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);

            //        GL.PopMatrix();
            //        index += 3;
            //    }
            //}

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

