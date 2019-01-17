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
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Helper
{
    internal sealed class LinearProblemBuilderEngine
    {
        #region Fields

        private readonly PhysicsEngineParameters EngineParameters;

        #endregion

        #region Constructor

        public LinearProblemBuilderEngine(PhysicsEngineParameters engineParameters)
        {
            EngineParameters = engineParameters;
        }

        #endregion

        #region Public Methods
                
        public LinearProblemProperties BuildLCP(
            JacobianConstraint[] constraints,
            double timeStep)
        {
            LinearProblemBaseProperties baseProperties = new LinearProblemBaseProperties(constraints.Length);

            Dictionary<HashSetStruct, List<DictionaryConstraintValue>> constraintsDictionary = new Dictionary<HashSetStruct, List<DictionaryConstraintValue>>();

            for (int i = 0; i < constraints.Length; i++)
            {
                JacobianConstraint itemConstraint = constraints[i];

                HashSetStruct hash = new HashSetStruct(itemConstraint.ObjectA.ID, itemConstraint.ObjectB.ID);

                if (constraintsDictionary.TryGetValue(hash, out List<DictionaryConstraintValue> jc))
                    jc.Add(new DictionaryConstraintValue(itemConstraint, i));
                else
                    constraintsDictionary.Add(hash, new List<DictionaryConstraintValue> { new DictionaryConstraintValue(itemConstraint, i) });
            }

            Graph graph = new Graph(constraints.Length);

            var dictionaryArray = constraintsDictionary.ToArray();

            var key_ID_A = constraintsDictionary.ToLookup(x => x.Key.ID_A);
            var key_ID_B = constraintsDictionary.ToLookup(x => x.Key.ID_B);

            var rangePartitioner = Partitioner.Create(
                0,
                dictionaryArray.Length,
                Convert.ToInt32(dictionaryArray.Length / EngineParameters.MaxThreadNumber) + 1);

            double timeFreq = 1.0 / timeStep;

            Parallel.ForEach(
                rangePartitioner,
                new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                (range, loopState) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        var constraintValues = dictionaryArray[i];
                        var constraintValueKey = constraintValues.Key;

                        int contactA_ID_A = constraintValues.Key.ID_A;
                        int contactA_ID_B = constraintValues.Key.ID_B;

                        for (int w = 0; w < constraintValues.Value.Count; w++)
                        {
                            JacobianConstraint contactA = constraintValues.Value[w].Constraint;

                            List<int> index = new List<int>();
                            List<double> values = new List<double>();

                            int indexVal = constraintValues.Value[w].Index;

                            double correctionVal = contactA.CorrectionValue * timeFreq;

                            double correctionValue = (correctionVal) < 0 ?
                                                     Math.Max(correctionVal, -EngineParameters.MaxCorrectionValue) :
                                                     Math.Min(correctionVal, EngineParameters.MaxCorrectionValue);

                            baseProperties.B[indexVal] = correctionValue - contactA.JacobianVelocity - contactA.ConstraintValue;
                            baseProperties.ConstraintsArray[indexVal] = contactA.ContactReference;
                            baseProperties.ConstraintLimit[indexVal] = contactA.ConstraintLimit;
                            baseProperties.ConstraintType[indexVal] = contactA.Type;

                            //Diagonal value
                            double mValue = GetLCPDiagonalValue(contactA) +
                                        (contactA.CFM * timeFreq) +
                                        EngineParameters.CFM +
                                        1E-40;

                            baseProperties.D[indexVal] = mValue;
                            baseProperties.InvD[indexVal] = 1.0 / mValue;

                            //contactA_ID_A == contactB_ID_A && contactA_ID_B == contactB_ID_B
                            for (int j = 0; j < constraintValues.Value.Count; j++)
                            {
                                int innerIndex = constraintValues.Value[j].Index;

                                if (innerIndex != indexVal)
                                {
                                    JacobianConstraint contactB = constraintValues.Value[j].Constraint;

                                    mValue = GetLCPMatrixValue(
                                        contactA.LinearComponentA,
                                        contactB.LinearComponentA,
                                        contactA.AngularComponentA,
                                        contactB.AngularComponentA,
                                        contactA.ObjectA.MassInfo);

                                    mValue += GetLCPMatrixValue(
                                            contactA.LinearComponentB,
                                            contactB.LinearComponentB,
                                            contactA.AngularComponentB,
                                            contactB.AngularComponentB,
                                            contactA.ObjectB.MassInfo);

                                    AddValues(ref index, ref values, mValue, innerIndex);
                                }
                            }

                            //contactA_ID_A == contactB_ID_B && contactA_ID_B == contactB_ID_A
                            var symmetricHashSet = new HashSetStruct(contactA_ID_B, contactA_ID_A);
                            if (constraintsDictionary.TryGetValue(symmetricHashSet, out List<DictionaryConstraintValue> symmetricList))
                            {
                                foreach (var item in symmetricList)
                                {
                                    int innerIndex = item.Index;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = item.Constraint;

                                        mValue = GetLCPMatrixValue(
                                            contactA.LinearComponentA,
                                            contactB.LinearComponentB,
                                            contactA.AngularComponentA,
                                            contactB.AngularComponentB,
                                            contactA.ObjectA.MassInfo);

                                        mValue += GetLCPMatrixValue(
                                            contactA.LinearComponentB,
                                            contactB.LinearComponentA,
                                            contactA.AngularComponentB,
                                            contactB.AngularComponentA,
                                            contactA.ObjectB.MassInfo);

                                        AddValues(ref index, ref values, mValue, innerIndex);
                                    }
                                }
                            }

                            //contactA_ID_A == contactB_ID_A
                            foreach (var constraintCheckItem in key_ID_A[contactA_ID_A])
                            {
                                AddLCPValues(
                                    ref index,
                                    ref values,
                                    indexVal,
                                    constraintCheckItem,
                                    constraintValueKey,
                                    symmetricHashSet,
                                    contactA.LinearComponentA,
                                    contactA.AngularComponentA,
                                    contactA.ObjectA.MassInfo,
                                    true);
                            }

                            //contactA_ID_A == contactB_ID_B
                            foreach (var constraintCheckItem in key_ID_B[contactA_ID_A])
                            {
                                AddLCPValues(
                                    ref index,
                                    ref values,
                                    indexVal,
                                    constraintCheckItem,
                                    constraintValueKey,
                                    symmetricHashSet,
                                    contactA.LinearComponentA,
                                    contactA.AngularComponentA,
                                    contactA.ObjectA.MassInfo,
                                    false);
                            }

                            //contactA_ID_B == contactB_ID_A
                            foreach (var constraintCheckItem in key_ID_A[contactA_ID_B])
                            {
                                AddLCPValues(
                                    ref index,
                                    ref values,
                                    indexVal,
                                    constraintCheckItem,
                                    constraintValueKey,
                                    symmetricHashSet,
                                    contactA.LinearComponentB,
                                    contactA.AngularComponentB,
                                    contactA.ObjectB.MassInfo,
                                    true);
                            }

                            //contactA_ID_B == contactB_ID_B
                            foreach (var constraintCheckItem in key_ID_B[contactA_ID_B])
                            {
                                AddLCPValues(
                                    ref index,
                                    ref values,
                                    indexVal,
                                    constraintCheckItem,
                                    constraintValueKey,
                                    symmetricHashSet,
                                    contactA.LinearComponentB,
                                    contactA.AngularComponentB,
                                    contactA.ObjectB.MassInfo,
                                    false);
                            }

                            //Sparse Matrix
                            baseProperties.M.Rows[indexVal] = new SparseVector(
                                                                values.ToArray(),
                                                                index.ToArray(),
                                                                baseProperties.M.m);

                            UpdateGraph(ref graph, index, indexVal);
                        }
                    }
                });

            var lp = new LinearProblemProperties(
                baseProperties,
                graph);
                        
            return new LinearProblemProperties(
                baseProperties,
                graph);
        }

        //TODO Refactor
        public LinearProblemProperties GetLCPFrictionMatrix(LinearProblemProperties linearProblem)
        {
            var collisionIndexes = linearProblem.ConstraintType
                                                .Select((constraint, index) => new { constraint, index })
                                                .Where(x => x.constraint == ConstraintType.Collision).ToList();

            if (collisionIndexes.Count > 0)
            {
                var numberOfCollision = collisionIndexes.Count;
                var lcpMatrixDim = linearProblem.Count + numberOfCollision;

                var frictionIndexes = linearProblem.ConstraintType
                                                   .Select((constraint, index) => new { constraint, index })
                                                   .Where(x => x.constraint == ConstraintType.Friction).ToList();

                LinearProblemBaseProperties baseProperties = new LinearProblemBaseProperties(lcpMatrixDim);

                var b = linearProblem.B.ToList();
                var d = linearProblem.D.ToList();
                var invD = linearProblem.InvD.ToList();
                var constraintType = linearProblem.ConstraintType.ToList();
                var constraintLimit = linearProblem.ConstraintLimit.ToList();
                var m = linearProblem.GetOriginalSparseMatrix().Rows.ToList();
                var constraintsArray = linearProblem.Constraints.ToList();
                var startValue = linearProblem.StartImpulse.ToList();
                var E_dim = numberOfCollision * EngineParameters.FrictionDirections;
                var newRowLength = b.Count + numberOfCollision;
                                
                //Add fields to friction rows
                for (int i = 0; i < numberOfCollision; i++)
                {
                    var colIdx = collisionIndexes[i].index;
                    
                    var fIndexes = constraintsArray.Select((constraint, index) => new { constraint, index })
                                                   .Where(x => x.constraint.HasValue && x.constraint.Value == colIdx).ToList();
                                        
                    //Add elements to matrix M, friction values
                    var rowIndexes = new List<int>() { collisionIndexes[i].index };
                    var rowValues = new List<double>() { constraintLimit[frictionIndexes[0].index] };
                                        
                    //Columns and rows update
                    for (int j = 0; j < fIndexes.Count; j++)
                    {
                        var idx = fIndexes[j].index;
                        var mIdx = m[idx];
                        var bufIndex = new List<int>(mIdx.Index);
                        var bufValue = new List<double>(mIdx.Value);
                        bufIndex.Add(mIdx.Length + i);
                        bufValue.Add(1.0);

                        m[idx] = new SparseVector(
                            bufValue.ToArray(),
                            bufIndex.ToArray(),
                            mIdx.Length + numberOfCollision);

                        rowIndexes.Add(idx);
                        rowValues.Add(-1.0);
                    }

                    //Rows Update
                    constraintType.Add(ConstraintType.FrictionValue);
                    b.Add(0.0);
                    d.Add(0.0);
                    invD.Add(0.0);
                    constraintLimit.Add(0.0);
                    startValue.Add(0.0);                    
                    
                    var sparseElement = new SparseVector(
                        rowValues.ToArray(), 
                        rowIndexes.ToArray(), 
                        newRowLength);

                    m.Add(sparseElement);
                }

                for (int i = 0; i < m.Count; i++)
                {
                    m[i] = new SparseVector(m[i].Value, m[i].Index, newRowLength);
                    b[i] *= -1.0; 
                }

                baseProperties.B = b.ToArray();
                baseProperties.D = d.ToArray();
                baseProperties.InvD = invD.ToArray();
                baseProperties.ConstraintType = constraintType.ToArray();
                baseProperties.ConstraintLimit = constraintLimit.ToArray();
                baseProperties.StartValue = startValue.ToArray();
                baseProperties.M.Rows = m.ToArray();

                var lcp = new LinearProblemProperties(baseProperties, null);

                //TODO Test
                //var test = lcp.GetOriginalMatrix();

                return lcp;
            }

            return linearProblem;
        }

        #endregion

        #region Private Methods

        private void UpdateGraph(
            ref Graph graph,
            List<int> index,
            int indexVal)
        {
            var lst = new List<int>(index);
            lst.Add(indexVal);
            graph.AddEdge(indexVal, lst);
        }

        private void AddLCPValues(
            ref List<int> index,
            ref List<double> values,
            int indexVal,
            KeyValuePair<HashSetStruct, List<DictionaryConstraintValue>> constraintCheckItem,
            HashSetStruct constraintValuesKey,
            HashSetStruct symmetricHashSet,
            Vector3d? linearComponentA,
            Vector3d angularComponentA,
            MassData massData, 
            bool componentA)
        {
            double mValue = 0.0;

            if (!constraintCheckItem.Key.Equals(constraintValuesKey) &&
                !constraintCheckItem.Key.Equals(symmetricHashSet))
            {
                for (int i = 0; i < constraintCheckItem.Value.Count; i++)
                {
                    int innerIndex = constraintCheckItem.Value[i].Index;

                    JacobianConstraint contactB = constraintCheckItem.Value[i].Constraint;

                    var lComponent = contactB.LinearComponentB;
                    var aComponent = contactB.AngularComponentB;

                    if (componentA)
                    {
                        lComponent = contactB.LinearComponentA;
                        aComponent = contactB.AngularComponentA;
                    }

                    mValue = GetLCPMatrixValue(
                        linearComponentA,
                        lComponent,
                        angularComponentA,
                        aComponent,
                        massData);

                    AddValues(ref index, ref values, mValue, innerIndex);
                }
            }
        }

        private double GetLCPMatrixValue(
            Vector3d? linearComponentA,
            Vector3d? linearComponentB,
            Vector3d angularComponentA,
            Vector3d angularComponentB,
            MassData massData)
        {
            double LCPValue = 0.0;

            if (linearComponentA.HasValue &&
                linearComponentB.HasValue)
                LCPValue += linearComponentA.Value.Dot(
                            linearComponentB.Value * massData.InverseMass);

            LCPValue += angularComponentA.Dot(
                        massData.InverseInertiaTensor * angularComponentB);

            return LCPValue;
        }


        private double GetLCPDiagonalValue(JacobianConstraint contact)
        {
            double LCPvalue = 0.0;

            if (contact.LinearComponentA.HasValue)
                LCPvalue += contact.LinearComponentA.Value.Dot(
                            contact.LinearComponentA.Value * contact.ObjectA.MassInfo.InverseMass);

            LCPvalue += contact.AngularComponentA.Dot(
                        contact.ObjectA.MassInfo.InverseInertiaTensor * contact.AngularComponentA);

            if (contact.LinearComponentB.HasValue)
                LCPvalue += contact.LinearComponentB.Value.Dot(
                            contact.LinearComponentB.Value * contact.ObjectB.MassInfo.InverseMass);

            LCPvalue += contact.AngularComponentB.Dot(
                        contact.ObjectB.MassInfo.InverseInertiaTensor * contact.AngularComponentB);

            return LCPvalue;
        }

        private static void AddValues(
            ref List<int> index,
            ref List<double> values,
            double mValue,
            int innerIndex)
        {
            if (Math.Abs(mValue) > 1E-16)
            {
                index.Add(innerIndex);
                values.Add(mValue);
            }
        }

        #endregion

    }
}
