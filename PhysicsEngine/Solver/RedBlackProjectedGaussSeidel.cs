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

        private enum RedBlackEnum
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
            redBlackDictionary.TryGetValue(RedBlackEnum.Red, out List<int> red);
            redBlackDictionary.TryGetValue(RedBlackEnum.Black, out List<int> black);

            var rangePartitionerBlack = Partitioner.Create(0, black.Count, Convert.ToInt32(black.Count / SolverParameters.MaxThreadNumber) + 1);
            var rangePartitionerRed = Partitioner.Create(0, red.Count, Convert.ToInt32(red.Count / SolverParameters.MaxThreadNumber) + 1);
                        
            for (int k = 0; k < SolverParameters.MaxIteration; k++)
            {
                //Execute Red
                Parallel.ForEach(
                    rangePartitionerRed,
                    new ParallelOptions { MaxDegreeOfParallelism = SolverParameters.MaxThreadNumber },
                    (range, loopState) =>
                    {
                        for (int i = range.Item1; i < range.Item2; i++)
                            Execute(input, red[i], ref x);
                    });

                //Execute Black
                Parallel.ForEach(
                    rangePartitionerBlack,
                    new ParallelOptions { MaxDegreeOfParallelism = SolverParameters.MaxThreadNumber },
                    (range, loopState) =>
                    {
                        for (int i = range.Item1; i < range.Item2; i++)
                            Execute(input, black[i], ref x);
                    });
            }

            return x;
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

        private Dictionary<RedBlackEnum, List<int>> GetRedBlackDictionary(LinearProblemProperties input)
        {
            var graph = input.ConstrGraph;
            var algorithms = new BreadthFirstSearch();

            var nodeDictionary = algorithms.GetLevelBFS(graph, 0);

            if (nodeDictionary.Count < input.Count)
            {
                for (int i = 0; i < input.Count; i++)
                {
                    if (!nodeDictionary.ContainsKey(i))
                    {
                        var dict = algorithms.GetLevelBFS(graph, i);
                        foreach (var element in dict)
                            nodeDictionary.Add(element.Key, element.Value);

                        if (nodeDictionary.Count == input.Count)
                            break;
                    }
                }
            }

            Dictionary<RedBlackEnum, List<int>> redBlackDictionary = GetRedBlackDictionary(nodeDictionary);
            
            return redBlackDictionary;
        }
        
        private Dictionary<RedBlackEnum, List<int>> GetRedBlackDictionary(Dictionary<int, int> nodeDictionary)
        {
            Dictionary<RedBlackEnum, List<int>> redBlackDictionary = new Dictionary<RedBlackEnum, List<int>>
            {
                { RedBlackEnum.Red, new List<int>() },
                { RedBlackEnum.Black, new List<int>() }
            };

            for (int i = 0; i < nodeDictionary.Count; i++)
            {
                if (nodeDictionary.TryGetValue(i, out int value))
                {
                    if (value % 2 == 0)
                    {
                        redBlackDictionary.TryGetValue(RedBlackEnum.Black, out List<int> list);
                        list.Add(i);
                    }
                    else
                    {
                        redBlackDictionary.TryGetValue(RedBlackEnum.Red, out List<int> list);
                        list.Add(i);
                    }
                }
            }
            
            return redBlackDictionary;
        }

        private List<HashSetStruct> GetEdges(SparseElement[] elements)
        {
            List<HashSetStruct> edges = new List<HashSetStruct>();

            for (int i = 0; i < elements.Length; i++)
                for (int j = 0; j < elements[i].Count; j++)
                    edges.Add(new HashSetStruct(i, elements[i].Index[j]));
            
            return edges;
        }

        private void Execute(
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
