using SharpEngineMathUtility;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Helper
{
    public sealed class LinearProblemBuilder
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

        public LinearProblemProperties BuildLCP(
            JacobianConstraint[] constraint,
            bool positionStabilization = false)
        {
            if (constraint.Length > 0)
            {
                double[] B = new double[constraint.Length];
                double[] D = new double[constraint.Length];
                ConstraintType[] constraintsType = new ConstraintType[constraint.Length];
                double[] constraintsLimit = new double[constraint.Length];
                SparseElement[] M = new SparseElement[constraint.Length];
                int?[] constraintsArray = new int?[constraint.Length];

                Dictionary<HashSetStruct, List<DictionaryConstraintValue>> constraintsDictionary = new Dictionary<HashSetStruct, List<DictionaryConstraintValue>>();

                for (int i = 0; i < constraint.Length; i++)
                {
                    JacobianConstraint itemConstraint = constraint[i];

                    List<DictionaryConstraintValue> jc;

                    HashSetStruct hash = new HashSetStruct(itemConstraint.ObjectA.ID, itemConstraint.ObjectB.ID);

                    if (constraintsDictionary.TryGetValue(hash, out jc))
                        jc.Add(new DictionaryConstraintValue(itemConstraint, i));
                    else
                        constraintsDictionary.Add(hash, new List<DictionaryConstraintValue> { new DictionaryConstraintValue(itemConstraint, i) });
                }
                                
                var dictionaryArray = constraintsDictionary.ToArray();

                var key_ID_A = constraintsDictionary.ToLookup(x => x.Key.ID_A);
                var key_ID_B = constraintsDictionary.ToLookup(x => x.Key.ID_B);

                Parallel.ForEach(
                    dictionaryArray, 
                    new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                    constraintValues =>
                {
                    int contactA_ID_A = constraintValues.Key.ID_A;
                    int contactA_ID_B = constraintValues.Key.ID_B;
                                                                               
                    for (int w = 0; w < constraintValues.Value.Count; w++)
                    {
                        JacobianConstraint contactA = constraintValues.Value[w].Constraint;

                        List<int> index = new List<int>();
                        List<double> values = new List<double>();

                        int indexVal = constraintValues.Value[w].Index;

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
                        List<DictionaryConstraintValue> symmetricList;
                        var symmetricHashSet = new HashSetStruct(contactA_ID_B, contactA_ID_A);
                        if (constraintsDictionary.TryGetValue(symmetricHashSet, out symmetricList))
                        {
                            for (int j = 0; j < symmetricList.Count; j++)
                            {
                                int innerIndex = symmetricList[j].Index;

                                if (innerIndex != indexVal)
                                {
                                    JacobianConstraint contactB = symmetricList[j].Constraint;

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
                            if (!constraintCheckItem.Key.Equals(constraintValues.Key) &&
                                !constraintCheckItem.Key.Equals(symmetricHashSet))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value[j].Index;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value[j].Constraint;

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
                            if (!constraintCheckItem.Key.Equals(constraintValues.Key) &&
                                !constraintCheckItem.Key.Equals(symmetricHashSet))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value[j].Index;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value[j].Constraint;

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
                            if (!constraintCheckItem.Key.Equals(constraintValues.Key) &&
                                !constraintCheckItem.Key.Equals(symmetricHashSet))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value[j].Index;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value[j].Constraint;

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
                            if (!constraintCheckItem.Key.Equals(constraintValues.Key) &&
                                !constraintCheckItem.Key.Equals(symmetricHashSet))
                            {
                                for (int j = 0; j < constraintCheckItem.Value.Count; j++)
                                {
                                    int innerIndex = constraintCheckItem.Value[j].Index;

                                    if (innerIndex != indexVal)
                                    {
                                        JacobianConstraint contactB = constraintCheckItem.Value[j].Constraint;

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
        public LinearProblemProperties OldBuildLCP(
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
            Vector3? linearComponentA,
            Vector3? linearComponentB,
            Vector3 angularComponentA,
            Vector3 angularComponentB,
            double inverseMass,
            Matrix3x3 inertiaTensor)
        {
            double LCPValue = 0.0;

            if(linearComponentA.HasValue &&
               linearComponentB.HasValue)
                LCPValue += linearComponentA.Value.Dot(
                            linearComponentB.Value * inverseMass);

            LCPValue += angularComponentA.Dot(
                        inertiaTensor * angularComponentB);

            return LCPValue;
        }


        private double GetLCPDiagonalValue(JacobianConstraint contact)
        {
            double LCPvalue = 0.0;

            if(contact.LinearComponentA.HasValue)
                LCPvalue +=  contact.LinearComponentA.Value.Dot(
                          contact.LinearComponentA.Value * contact.ObjectA.InverseMass);

            LCPvalue += contact.AngularComponentA.Dot(
                     contact.ObjectA.InertiaTensor * contact.AngularComponentA);

            if(contact.LinearComponentB.HasValue)
                LCPvalue += contact.LinearComponentB.Value.Dot(
                          contact.LinearComponentB.Value * contact.ObjectB.InverseMass);

            LCPvalue += contact.AngularComponentB.Dot(
                    contact.ObjectB.InertiaTensor * contact.AngularComponentB);

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
