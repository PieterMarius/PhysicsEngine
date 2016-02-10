using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GaussSeidel
{
    class Program
    {
        static void Main(string[] args)
        {

            GaussSeidel solver = new GaussSeidel(1.0);

            double[] M = new double[9];
            M[0] = 12.0;
            M[1] = 3.0;
            M[2] = -5.0;
            M[3] = 1.0;
            M[4] = 5.0;
            M[5] = 3.0;
            M[6] = 3.0;
            M[7] = 7.0;
            M[8] = 13.0;
            

            double[] B = new double[3];
            B[0] = 1.0;
            B[1] = 28.0;
            B[2] = 76.0;
            
            double[] startX = new double[3];
            startX[0] = 1.0;
            startX[0] = 0.0;
            startX[0] = 1.0;

            LinearProblemProperties linearProblemProperties = new LinearProblemProperties(
                M,
                B,
                startX,
                3);

            double[] X = new double[3];

            NonLinearConjugateGradient nonLinearConjugateGradient = new NonLinearConjugateGradient(3);

            double[] X1 = nonLinearConjugateGradient.Solve(linearProblemProperties);

            for (int i= 0;i<4;i++)
                X = solver.Solve(linearProblemProperties);

            double[] out0 = nonLinearConjugateGradient.CalculateError(linearProblemProperties.M, linearProblemProperties.B, X1);
           double[] out1 = nonLinearConjugateGradient.CalculateError(linearProblemProperties.M, linearProblemProperties.B, X);

            Console.ReadLine();


        }

       
    }

}
