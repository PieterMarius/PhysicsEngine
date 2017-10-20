using SharpEngineMathUtility;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine
{
    public sealed class LinearProblemBuilder
    {
        #region Fields

        private struct HashSetStruct : IEquatable<HashSetStruct>
        {
            private readonly int id_a, id_b;

            public int ID_A { get { return id_a; } }
            public int ID_B { get { return id_b; } }

            public HashSetStruct(int id_a, int id_b)
            {
                this.id_a = id_a;
                this.id_b = id_b;
            }

            public override int GetHashCode()
            {
                return 31 * id_a + 17 * id_b;
            }

            public override bool Equals(object obj)
            {
                return obj is HashSetStruct && Equals((HashSetStruct)obj);
            }

            public bool Equals(HashSetStruct p)
            {
                return id_a == p.ID_A && id_b == p.ID_B;
            }

        }

        private readonly PhysicsEngineParameters EngineParameters;

        #endregion

        #region Constructor

        public LinearProblemBuilder(PhysicsEngineParameters engineParameters)
        {
            EngineParameters = engineParameters;
        }

        #endregion

        #region Public Methods

        public LinearProblemProperties BuildLCPMatrix(
            JacobianConstraint[] constraint,
            bool positionStabilization = false)
        {
            if (constraint.Length > 0)
            {
                var stopwatch = new Stopwatch();

                double[] B = new double[constraint.Length];
                double[] D = new double[constraint.Length];
                ConstraintType[] constraintsType = new ConstraintType[constraint.Length];
                double[] constraintsLimit = new double[constraint.Length];
                SparseElement[] M = new SparseElement[constraint.Length];
                int?[] constraintsArray = new int?[constraint.Length];
                                                               
                Dictionary<HashSetStruct, Tuple<List<JacobianConstraint>, int>> constraintsDictionary = new Dictionary<HashSetStruct, Tuple<List<JacobianConstraint>, int>>();
                               
                for (int i = 0; i < constraint.Length; i++)
                {
                    JacobianConstraint itemConstraint = constraint[i];

                    Tuple<List<JacobianConstraint>, int> jc;

                    HashSetStruct hash = new HashSetStruct(itemConstraint.ObjectA.GetID(), itemConstraint.ObjectB.GetID());

                    if (constraintsDictionary.TryGetValue(hash, out jc))
                    {
                        jc.Item1.Add(itemConstraint);

                        jc = new Tuple<List<JacobianConstraint>, int>(jc.Item1, i);
                    }
                    else
                        constraintsDictionary.Add(hash, new Tuple<List<JacobianConstraint>, int>(new List<JacobianConstraint> { itemConstraint }, i));
                }
                
                var dictionaryList = constraintsDictionary.ToArray();

                var key_ID_A = constraintsDictionary.ToLookup(x => x.Key.ID_A);
                var key_ID_B = constraintsDictionary.ToLookup(x => x.Key.ID_B);

                Parallel.ForEach(dictionaryList, new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    constraintList =>
                {
                    int contactA_ID_A = constraintList.Key.ID_A;
                    int contactA_ID_B = constraintList.Key.ID_B;
                                        
                    for (int w = 0; w < constraintList.Value.Item1.Count; w++)
                    {
                        JacobianConstraint contactA = constraintList.Value.Item1[w];

                        List<int> index = new List<int>();
                        List<double> values = new List<double>();

                        int indexVal = constraintList.Value.Item2 + w;

                        if (positionStabilization)
                            B[indexVal] = contactA.CorrectionValue;
                        else
                            B[indexVal] = -(contactA.B - ((contactA.CorrectionValue) < 0 ? Math.Max(contactA.CorrectionValue, -EngineParameters.MaxCorrectionValue) :
                                                                                           Math.Min(contactA.CorrectionValue, EngineParameters.MaxCorrectionValue)));
                        
                        constraintsArray[indexVal] = contactA.ContactReference;

                        constraintsLimit[indexVal] = contactA.ConstraintLimit;
                        constraintsType[indexVal] = contactA.Type;

                        double mValue = GetLCPDiagonalValue(contactA);

                        //Diagonal value
                        mValue += contactA.CFM +
                                  EngineParameters.CFM +
                                  1E-40;

                        D[indexVal] = 1.0 / mValue;

                        //contactA_ID_A == contactB_ID_A && contactA_ID_B == contactB_ID_B
                        for (int j = 0; j < constraintList.Value.Item1.Count; j++)
                        {
                            int innerIndex = constraintList.Value.Item2 + j;

                            if (innerIndex != indexVal)
                            {
                                JacobianConstraint contactB = constraintList.Value.Item1[j];

                                mValue = GetLCPMatrixValue(
                                    contactA.LinearComponentA,
                                    contactB.LinearComponentA,
                                    contactA.AngularComponentA,
                                    contactB.AngularComponentA,
                                    contactA.ObjectA.InverseMass,
                                    contactA.ObjectA.InertiaTensor);

                                mValue += GetLCPMatrixValue(
                                        contactA.LinearComponentB,
                                        contactB.LinearComponentB,
                                        contactA.AngularComponentB,
                                        contactB.AngularComponentB,
                                        contactA.ObjectB.InverseMass,
                                        contactA.ObjectB.InertiaTensor);

                                AddValues(ref index, ref values, mValue, innerIndex);
                            }
                        }

                        //contactA_ID_A == contactB_ID_B && contactA_ID_B == contactB_ID_A
                        Tuple<List<JacobianConstraint>, int> symmetricList;
                        if (constraintsDictionary.TryGetValue(new HashSetStruct(contactA_ID_B, contactA_ID_A), out symmetricList))
                        {
                            for (int j = 0; j < symmetricList.Item1.Count; j++)
                            {
                                int innerIndex = constraintList.Value.Item2 + j;

                                if (innerIndex != indexVal)
                                {
                                    JacobianConstraint contactB = symmetricList.Item1[j];

                                    mValue = GetLCPMatrixValue(
                                        contactA.LinearComponentA,
                                        contactB.LinearComponentB,
                                        contactA.AngularComponentA,
                                        contactB.AngularComponentB,
                                        contactA.ObjectA.InverseMass,
                                        contactA.ObjectA.InertiaTensor);

                                    mValue += GetLCPMatrixValue(
                                        contactA.LinearComponentB,
                                        contactB.LinearComponentA,
                                        contactA.AngularComponentB,
                                        contactB.AngularComponentA,
                                        contactA.ObjectB.InverseMass,
                                        contactA.ObjectB.InertiaTensor);

                                    AddValues(ref index, ref values, mValue, innerIndex);
                                }
                            }
                        }

                        //contactA_ID_A == contactB_ID_A
                        foreach (var constraintCheckItem in key_ID_A[contactA_ID_A])
                        {
                            if (!constraintCheckItem.Key.Equals(constraintList.Key))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Item1.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value.Item2 + j;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value.Item1[j];

                                        mValue = GetLCPMatrixValue(
                                            contactA.LinearComponentA,
                                            contactB.LinearComponentA,
                                            contactA.AngularComponentA,
                                            contactB.AngularComponentA,
                                            contactA.ObjectA.InverseMass,
                                            contactA.ObjectA.InertiaTensor);

                                        AddValues(ref index, ref values, mValue, innerIndex);
                                    }
                                }
                            }
                        }

                        //contactA_ID_A == contactB_ID_B
                        foreach (var constraintCheckItem in key_ID_B[contactA_ID_A])
                        {
                            if (!constraintCheckItem.Key.Equals(constraintList.Key))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Item1.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value.Item2 + j;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value.Item1[j];

                                        mValue = GetLCPMatrixValue(
                                            contactA.LinearComponentA,
                                            contactB.LinearComponentB,
                                            contactA.AngularComponentA,
                                            contactB.AngularComponentB,
                                            contactA.ObjectA.InverseMass,
                                            contactA.ObjectA.InertiaTensor);

                                        AddValues(ref index, ref values, mValue, innerIndex);
                                    }
                                }
                            }
                        }

                        //contactA_ID_B == contactB_ID_A
                        foreach (var constraintCheckItem in key_ID_A[contactA_ID_B])
                        {
                            if (!constraintCheckItem.Key.Equals(constraintList.Key))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Item1.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value.Item2 + j;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value.Item1[j];

                                        mValue = GetLCPMatrixValue(
                                            contactA.LinearComponentB,
                                            contactB.LinearComponentA,
                                            contactA.AngularComponentB,
                                            contactB.AngularComponentA,
                                            contactA.ObjectB.InverseMass,
                                            contactA.ObjectB.InertiaTensor);

                                        AddValues(ref index, ref values, mValue, innerIndex);
                                    }
                                }
                            }
                        }

                        //contactA_ID_B == contactB_ID_B
                        foreach (var constraintCheckItem in key_ID_B[contactA_ID_B])
                        {
                            if (!constraintCheckItem.Key.Equals(constraintList.Key))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Item1.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value.Item2 + j;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value.Item1[j];

                                        mValue = GetLCPMatrixValue(
                                            contactA.LinearComponentB,
                                            contactB.LinearComponentB,
                                            contactA.AngularComponentB,
                                            contactB.AngularComponentB,
                                            contactA.ObjectB.InverseMass,
                                            contactA.ObjectB.InertiaTensor);

                                        AddValues(ref index, ref values, mValue, innerIndex);
                                    }
                                }
                            }
                        }

                        //Sparse Matrix
                        M[indexVal] = new SparseElement(
                            values.ToArray(),
                            index.ToArray(),
                            constraint.Length);
                    }
                });

                return new LinearProblemProperties(
                    M,
                    B,
                    D,
                    constraintsLimit,
                    constraintsType,
                    constraintsArray);
            }

            return null;
        }

        


        /// <summary>
        /// Builds the LCP matrix for solver.
        /// </summary>
        [Obsolete]
        public LinearProblemProperties OldBuildLCPMatrix(
            JacobianConstraint[] constraint,
            bool positionStabilization = false)
        {
            if (constraint.Length > 0)
            {
                double[] B = new double[constraint.Length];
                double[] D = new double[constraint.Length];
                ConstraintType[] constraintsType = new ConstraintType[constraint.Length];
                double[] constraintsLimit = new double[constraint.Length];
                int?[] constraints = new int?[constraint.Length];

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

                        if (positionStabilization)
                            B[i] = contactA.CorrectionValue;
                        else
                            B[i] = -(contactA.B - ((contactA.CorrectionValue) < 0 ? Math.Max(contactA.CorrectionValue, -EngineParameters.MaxCorrectionValue) :
                                                                                    Math.Min(contactA.CorrectionValue, EngineParameters.MaxCorrectionValue)));

                        constraints[i] = contactA.ContactReference;

                        constraintsLimit[i] = contactA.ConstraintLimit;
                        constraintsType[i] = contactA.Type;

                        double mValue = GetLCPDiagonalValue(contactA);

                        //Diagonal value
                        mValue += contactA.CFM +
                                  EngineParameters.CFM +
                                  1E-40;

                        D[i] = 1.0 / mValue;

                        int contactA_ID_A = contactA.ObjectA.GetID();
                        int contactA_ID_B = contactA.ObjectB.GetID();

                        for (int j = i + 1; j < constraint.Length; j++)
                        {
                            JacobianConstraint contactB = constraint[j];

                            int contactB_ID_A = contactB.ObjectA.GetID();
                            int contactB_ID_B = contactB.ObjectB.GetID();

                            mValue = 0.0;

                            if (contactA_ID_A == contactB_ID_A)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentA,
                                    contactB.LinearComponentA,
                                    contactA.AngularComponentA,
                                    contactB.AngularComponentA,
                                    contactA.ObjectA.InverseMass,
                                    contactA.ObjectA.InertiaTensor);
                            }
                            else if (contactA_ID_A == contactB_ID_B)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentA,
                                    contactB.LinearComponentB,
                                    contactA.AngularComponentA,
                                    contactB.AngularComponentB,
                                    contactA.ObjectA.InverseMass,
                                    contactA.ObjectA.InertiaTensor);
                            }

                            if (contactA_ID_B == contactB_ID_A)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentB,
                                    contactB.LinearComponentA,
                                    contactA.AngularComponentB,
                                    contactB.AngularComponentA,
                                    contactA.ObjectB.InverseMass,
                                    contactA.ObjectB.InertiaTensor);
                            }
                            else if (contactA_ID_B == contactB_ID_B)
                            {
                                mValue += GetLCPMatrixValue(
                                    contactA.LinearComponentB,
                                    contactB.LinearComponentB,
                                    contactA.AngularComponentB,
                                    contactB.AngularComponentB,
                                    contactA.ObjectB.InverseMass,
                                    contactA.ObjectB.InertiaTensor);
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
                        index[i].ToArray(),
                        constraint.Length);
                }

                return new LinearProblemProperties(
                    M,
                    B,
                    D,
                    constraintsLimit,
                    constraintsType,
                    constraints);
            }

            return null;
        }

        #endregion

        #region Private Methods

        private double GetLCPMatrixValue(
            Vector3 linearComponentA,
            Vector3 linearComponentB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            double inverseMass,
            Matrix3x3 inertiaTensor)
        {
            double linear = linearComponentA.Dot(
                        linearComponentB * inverseMass);

            double angular = angularComponentA.Dot(
                        inertiaTensor * angularComponentB);

            return linear + angular;
        }


        private double GetLCPDiagonalValue(JacobianConstraint contact)
        {
            double linearA = contact.LinearComponentA.Dot(
                    contact.LinearComponentA * contact.ObjectA.InverseMass);

            double angularA = contact.AngularComponentA.Dot(
                    contact.ObjectA.InertiaTensor * contact.AngularComponentA);

            double linearB = contact.LinearComponentB.Dot(
                    contact.LinearComponentB * contact.ObjectB.InverseMass);

            double angularB = contact.AngularComponentB.Dot(
                    contact.ObjectB.InertiaTensor * contact.AngularComponentB);

            return (linearA + angularA) +
                   (linearB + angularB);
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
