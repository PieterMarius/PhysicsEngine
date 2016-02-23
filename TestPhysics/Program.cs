﻿using System;
using System.Collections.Generic;
using System.Text;
using OpenTK;
using PhysicsEngineMathUtility;
using SimulationObjectDefinition;
using Utility;
using CollisionEngine;
using ObjLoader;
using ObjLoader.Loader.Loaders;
using System.IO;
using LCPSolver;

namespace TestPhysics
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			
			using (TestWindow test = new TestWindow ()) 
			{
				
				//Implemetare o scaricare .dll algoritmo di caricamento dei file obj


				//GJKCollisionDetectionEngine collisionEngine = new GJKCollisionDetectionEngine (10, 0.1, 0.00001);
//				var objLoaderFactory = new ObjLoaderFactory ();
//				var objLoader = objLoaderFactory.Create ();
//				var fileStream = new FileStream ("dodeca.obj", FileMode.OpenOrCreate);
//				var result = objLoader.Load (fileStream);
//
//				System.Console.WriteLine("Nvertex " + result.Vertices.Count);
//
//				for (int i = 0; i < result.Vertices.Count; i++) {
//					Console.WriteLine("Vertex " + i + " " + result.Vertices[i].X + " " + result.Vertices[i].Y + " " + result.Vertices[i].Z);
//				}

				test.Run();
				
			};



//			GaussSeidel solver = new GaussSeidel(1.0);
//
//			int nVar = 3;
//
//			double[] M = new double[nVar * nVar];
//			M[0] = 12.0;
//			M[1] = 3.0;
//			M[2] = -5.0;
//			M[3] = 1.0;
//			M[4] = 5.0;
//			M[5] = 3.0;
//			M[6] = 3.0;
//			M[7] = 7.0;
//			M[8] = 13.0;
//
//			double[] B = new double[nVar];
//			B[0] = 1.0;
//			B[1] = 28.0;
//			B[2] = 76.0;
//
//			double[] diag = new double[nVar];
//			diag [0] = 1.0 / 12;
//			diag [1] = 1.0 / 5;
//			diag [2] = 1.0 / 13;
//
//			double[] startX = new double[nVar];
//			startX[0] = 0.0;
//			startX[0] = 0.0;
//			startX[0] = 0.0;
//
//			double[] constraintLimit = new double[nVar];
//			ConstraintType[] constraintType = new ConstraintType[nVar];
//			constraintType [0] = ConstraintType.Joint;
//			constraintType [1] = ConstraintType.Joint;
//			constraintType [2] = ConstraintType.Joint;
//
//			int[] constraints = new int[nVar];
//
//			LinearProblemProperties linearProblemProperties = new LinearProblemProperties(
//				M,
//				B,
//				startX,
//				diag,
//				constraintLimit,
//				constraintType,
//				constraints,
//				nVar);
//
//			double[] X = new double[nVar];
//
//			SolverParameters solverParameters = new SolverParameters (
//				                                    5,
//				                                    0.000001,
//				                                    1.0);
//
//
//			NonLinearConjugateGradient nonLinearConjugateGradient = new NonLinearConjugateGradient(solverParameters);
//
//			double[] X1 = nonLinearConjugateGradient.Solve(linearProblemProperties);
//
//			linearProblemProperties.SetStartValue (new double[nVar]);
//
//			for (int i = 0; i < 6; i++) 
//			{
//				X = solver.Solve (linearProblemProperties);
//				linearProblemProperties.SetStartValue (X);
//			}
//
//			double[] out0 = nonLinearConjugateGradient.CalculateError(linearProblemProperties.M, linearProblemProperties.B, X1);
//			double[] out1 = nonLinearConjugateGradient.CalculateError(linearProblemProperties.M, linearProblemProperties.B, X);
//
//			Console.ReadLine();
		}
	}
}
