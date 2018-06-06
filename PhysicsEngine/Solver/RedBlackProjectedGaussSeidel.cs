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

using SharpEngineMathUtility;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.LCPSolver
{
    internal sealed class RedBlackProjectedGaussSeidel: ISolver
    {
        #region Fields

        public enum RedBlackEnum
        {
            Red = 0,
            Black = 1,
        }

        public readonly SolverParameters SolverParameters;

        #endregion

        #region Constructor

        public RedBlackProjectedGaussSeidel(
            SolverParameters solverParameters)
        {
            SolverParameters = solverParameters;
        }

        #endregion

        #region Public Methods

        public double[] Solve(
            LinearProblemProperties input,
            double[] x)
        {
            var redBlackDictionary = GetRedBlackDictionary(input);
            return Execute(input, redBlackDictionary, x);
        }

        public double[] SolveExecute(
            LinearProblemProperties input,
            Dictionary<RedBlackEnum, List<int>> redBlackDictionary,
            double[] x)
        {
            return Execute(input, redBlackDictionary, x);
        }

        private Dictionary<RedBlackEnum, List<int>> GetRedBlackDictionary(Dictionary<int, bool> nodeDictionary)
        {
            Dictionary<RedBlackEnum, List<int>> redBlackDictionary = new Dictionary<RedBlackEnum, List<int>>
            {
                { RedBlackEnum.Red, new List<int>() },
                { RedBlackEnum.Black, new List<int>() }
            };

            for (int i = 0; i < nodeDictionary.Count; i++)
            {
                if (nodeDictionary[i])
                    redBlackDictionary[RedBlackEnum.Black].Add(i);
                else
                    redBlackDictionary[RedBlackEnum.Red].Add(i);
            }

            return redBlackDictionary;
        }

        public Dictionary<RedBlackEnum, List<int>> GetRedBlackDictionary(LinearProblemProperties input)
        {
            var graph = input.ConstrGraph;

            var nodeDictionary = BreadthFirstSearch.GetBoolLevelBFS(graph, 0);

            if (nodeDictionary.Count < input.Count)
            {
                for (int i = 0; i < input.Count; i++)
                {
                    if (!nodeDictionary.ContainsKey(i))
                    {
                        var dict = BreadthFirstSearch.GetBoolLevelBFS(graph, i);
                        foreach (var element in dict)
                            nodeDictionary.Add(element.Key, element.Value);

                        if (nodeDictionary.Count == input.Count)
                            break;
                    }
                }
            }

            var redBlackDictionary = GetRedBlackDictionary(nodeDictionary);

            return redBlackDictionary;
        }

        public void SetSuccessiveOverRelaxation(double SOR)
        {
            SolverParameters.SetSOR(SOR);
        }

        public SolverParameters GetSolverParameters()
        {
            return SolverParameters;
        }

        #endregion

        #region Private Methods

        private double[] Execute(
            LinearProblemProperties input,
            Dictionary<RedBlackEnum, List<int>> redBlackDictionary,
            double[] x)
        {
            redBlackDictionary.TryGetValue(RedBlackEnum.Red, out List<int> red);
            redBlackDictionary.TryGetValue(RedBlackEnum.Black, out List<int> black);

            OrderablePartitioner<Tuple<int, int>> rangePartitionerBlack = null;
            OrderablePartitioner<Tuple<int, int>> rangePartitionerRed = null;

            try
            {
                rangePartitionerBlack = Partitioner.Create(0, black.Count, Convert.ToInt32(black.Count / SolverParameters.MaxThreadNumber) + 1);                       
                rangePartitionerRed = Partitioner.Create(0, red.Count, Convert.ToInt32(red.Count / SolverParameters.MaxThreadNumber) + 1);

                for (int k = 0; k < SolverParameters.MaxIteration; k++)
                {
                    if (red.Any())
                    {
                        //Execute Red
                        Parallel.ForEach(
                            rangePartitionerRed,
                            new ParallelOptions { MaxDegreeOfParallelism = SolverParameters.MaxThreadNumber },
                            (range, loopState) =>
                            {
                                for (int i = range.Item1; i < range.Item2; i++)
                                    ExecuteKernel(input, red[i], ref x);
                            });
                    }

                    if (black.Any())
                    {
                        //Execute Black
                        Parallel.ForEach(
                            rangePartitionerBlack,
                            new ParallelOptions { MaxDegreeOfParallelism = SolverParameters.MaxThreadNumber },
                            (range, loopState) =>
                            {
                                for (int i = range.Item1; i < range.Item2; i++)
                                    ExecuteKernel(input, black[i], ref x);
                            });
                    }
                }
            }
            catch(Exception ex)
            {
                throw new Exception(ex.Message);
            }
            return x;
            
        }
               
        private void ExecuteKernel(
            LinearProblemProperties input,
            int index,
            ref double[] x)
        {
            SparseElement m = input.M[index];
            double xValue = x[index];

            double sumBuffer = 0.0;
            
            double[] bufValue = m.Value;
            int[] bufIndex = m.Index;

            for (int j = 0; j < m.Count; j++)
            {
                int idx = bufIndex[j];
                sumBuffer += bufValue[j] * x[idx];
            }
            
            sumBuffer = (input.B[index] - sumBuffer) * input.InvD[index];

            xValue += (sumBuffer - xValue) * SolverParameters.SOR;

            x[index] = ClampSolution.Clamp(input, xValue, x, index);
        }

        #endregion
    }
}
