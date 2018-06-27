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
    internal sealed class LinearProblemBuilder
    {
        #region Fields

        private readonly PhysicsEngineParameters EngineParameters;

        #endregion

        #region Constructor

        public LinearProblemBuilder(PhysicsEngineParameters engineParameters)
        {
            EngineParameters = engineParameters;
        }

        #endregion

        #region Public Methods
                
        public LinearProblemProperties BuildLCP(JacobianConstraint[] constraints)
        {
            if (constraints.Length > 0)
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

                                double correctionValue = (contactA.CorrectionValue) < 0 ? 
                                                         Math.Max(contactA.CorrectionValue, -EngineParameters.MaxCorrectionValue) :
                                                         Math.Min(contactA.CorrectionValue, EngineParameters.MaxCorrectionValue);

                                baseProperties.B[indexVal] = correctionValue - contactA.B;
                                baseProperties.ConstraintsArray[indexVal] = contactA.ContactReference;
                                baseProperties.ConstraintLimit[indexVal] = contactA.ConstraintLimit;
                                baseProperties.ConstraintType[indexVal] = contactA.Type;
                                baseProperties.StartValue[indexVal] = contactA.StartImpulse.StartImpulseValue;

                                //Diagonal value
                                double mValue = GetLCPDiagonalValue(contactA) +
                                                contactA.CFM +
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
                                baseProperties.M[indexVal] = new SparseElement(
                                    values.ToArray(),
                                    index.ToArray());

                                UpdateGraph(ref graph, ref index, indexVal);
                            }
                        }
                    });

                return new LinearProblemProperties(
                    baseProperties,
                    graph);
            }

            return null;
        }



        /// <summary>
        /// Builds the LCP matrix for solver.
        /// </summary>
        [Obsolete]
        public LinearProblemProperties OldBuildLCP(JacobianConstraint[] constraint)
        {
            if (constraint.Length > 0)
            {
                double[] B = new double[constraint.Length];
                double[] D = new double[constraint.Length];
                double[] invD = new double[constraint.Length];
                ConstraintType[] constraintsType = new ConstraintType[constraint.Length];
                double[] constraintsLimit = new double[constraint.Length];
                int?[] constraints = new int?[constraint.Length];
                double[] startImpulse = new double[constraint.Length];

                List<int>[] index = new List<int>[constraint.Length];
                List<double>[] value = new List<double>[constraint.Length];

                for (int i = 0; i < constraint.Length; i++)
                {
                    index[i] = new List<int>();
                    value[i] = new List<double>();

                }

                //Critical section variable
                var sync = new object();

                Parallel.For(0,
                    constraint.Length,
                    new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    i =>
                    {
                        JacobianConstraint contactA = constraint[i];

                        B[i] = -(contactA.B - ((contactA.CorrectionValue) < 0 ? Math.Max(contactA.CorrectionValue, -EngineParameters.MaxCorrectionValue) :
                                                                                    Math.Min(contactA.CorrectionValue, EngineParameters.MaxCorrectionValue)));

                        constraints[i] = contactA.ContactReference;

                        constraintsLimit[i] = contactA.ConstraintLimit;
                        constraintsType[i] = contactA.Type;
                        startImpulse[i] = contactA.StartImpulse.StartImpulseValue;

                        double mValue = GetLCPDiagonalValue(contactA);

                        //Diagonal value
                        mValue += contactA.CFM +
                                  EngineParameters.CFM +
                                  1E-40;

                        D[i] = mValue;
                        invD[i] = 1.0 / mValue;

                        int contactA_ID_A = contactA.ObjectA.ID;
                        int contactA_ID_B = contactA.ObjectB.ID;

                        for (int j = i + 1; j < constraint.Length; j++)
                        {
                            JacobianConstraint contactB = constraint[j];

                            int contactB_ID_A = contactB.ObjectA.ID;
                            int contactB_ID_B = contactB.ObjectB.ID;

                            mValue = 0.0;

                            if (contactA_ID_A == contactB_ID_A)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentA,
                                    contactB.LinearComponentA,
                                    contactA.AngularComponentA,
                                    contactB.AngularComponentA,
                                    contactA.ObjectA.MassInfo);
                            }
                            else if (contactA_ID_A == contactB_ID_B)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentA,
                                    contactB.LinearComponentB,
                                    contactA.AngularComponentA,
                                    contactB.AngularComponentB,
                                    contactA.ObjectA.MassInfo);
                            }

                            if (contactA_ID_B == contactB_ID_A)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentB,
                                    contactB.LinearComponentA,
                                    contactA.AngularComponentB,
                                    contactB.AngularComponentA,
                                    contactA.ObjectB.MassInfo);
                            }
                            else if (contactA_ID_B == contactB_ID_B)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentB,
                                    contactB.LinearComponentB,
                                    contactA.AngularComponentB,
                                    contactB.AngularComponentB,
                                    contactA.ObjectB.MassInfo);
                            }

                            if (Math.Abs(mValue) > 1E-32)
                            {
                                lock (sync)
                                {
                                    index[i].Add(j);
                                    value[i].Add(mValue);
                                    index[j].Add(i);
                                    value[j].Add(mValue);
                                }
                            }
                        }
                    });

                SparseElement[] M = new SparseElement[constraint.Length];

                for (int i = 0; i < constraint.Length; i++)
                {
                    M[i] = new SparseElement(
                        value[i].ToArray(),
                        index[i].ToArray());
                }

                return new LinearProblemProperties(
                    M,
                    B,
                    D,
                    invD,
                    constraintsLimit,
                    constraintsType,
                    null,
                    constraints,
                    startImpulse);
            }

            return null;
        }

        #endregion

        #region Private Methods

        private void UpdateGraph(
            ref Graph graph,
            ref List<int> index,
            int indexVal)
        {
            index.Add(indexVal);
            graph.AddEdge(indexVal, index);
        }

        private void AddLCPValues(
            ref List<int> index,
            ref List<double> values,
            int indexVal,
            KeyValuePair<HashSetStruct, List<DictionaryConstraintValue>> constraintCheckItem,
            HashSetStruct constraintValuesKey,
            HashSetStruct symmetricHashSet,
            Vector3? linearComponentA,
            Vector3 angularComponentA,
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
            Vector3? linearComponentA,
            Vector3? linearComponentB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
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
            if (Math.Abs(mValue) > 1E-32)
            {
                index.Add(innerIndex);
                values.Add(mValue);
            }
        }

        #endregion

    }
}
