using System;
using System.Collections.Generic;
using System.Linq;
using SharpEngineMathUtility;
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Helper
{
    internal sealed class WarmStartEngine
    {
        #region Fields

        private Dictionary<WarmStartHashSet, Tuple<bool, int, CollisionPointStructure>> collisionPointsCounterDictionary;
        private readonly PhysicsEngineParameters EngineParameters;

        #endregion

        #region Constructor

        public WarmStartEngine(PhysicsEngineParameters engineParameters)
        {
            collisionPointsCounterDictionary = new Dictionary<WarmStartHashSet, Tuple<bool, int, CollisionPointStructure>>();
            EngineParameters = engineParameters;
        }

        #endregion

        #region Public Methods

        public List<CollisionPointStructure> GetWarmStartedCollisionPoints(
            IShape[] shapes,
            List<CollisionPointStructure> previousCollisionPoints,
            List<CollisionPointStructure> actualCollisionPoints,
            Dictionary<int, StabilizationValues> previousShapesProperties)
        {
            UpdatePersistentCollisionCounter(
                previousCollisionPoints,
                actualCollisionPoints,
                previousShapesProperties,
                shapes);

            Dictionary<WarmStartHashSet, CollisionPointStructure> warmStartedCollisionPoints = new Dictionary<WarmStartHashSet, CollisionPointStructure>();

            int warmStartCounter = 20;


            foreach (var item in actualCollisionPoints)
            {
                WarmStartHashSet hashset = new WarmStartHashSet(
                    item.ObjectIndexA, 
                    item.ObjectIndexB);

                if (collisionPointsCounterDictionary.TryGetValue(hashset, out Tuple<bool, int, CollisionPointStructure> value) &&
                    value.Item2 >= warmStartCounter)
                {
                    
                        /*
                        var shapeA = shapes.First(x => x.ID == hashset.ID_A);
                        var shapeB = shapes.First(x => x.ID == hashset.ID_B);

                        var posDiffA = Vector3.Length(shapeA.Position - objA.Position);
                        var posDiffB = Vector3.Length(shapeB.Position - objB.Position);

                        var angDiffA = Quaternion.Length(shapeA.RotationStatus - objA.RotationStatus);
                        var angDiffB = Quaternion.Length(shapeB.RotationStatus - objB.RotationStatus);


                        var velDiffA = Vector3.Length(shapeA.LinearVelocity - objA.LinearVelocity);
                        var velDiffB = Vector3.Length(shapeB.LinearVelocity - objB.LinearVelocity);

                        var angVelDiffA = Vector3.Length(shapeA.AngularVelocity - objA.AngularVelocity);
                        var angVelDiffB = Vector3.Length(shapeB.AngularVelocity - objB.AngularVelocity);
                        */

                        /*
                         if (posDiffA < distTolerance && posDiffB < distTolerance &&
                            angDiffA < distTolerance && angDiffB < distTolerance /*&&
                            velDiffA < velTolerance && velDiffB < velTolerance &&
                            angVelDiffA < velTolerance && angVelDiffB < velTolerance)
                        {
                        */
                        

                        warmStartedCollisionPoints.Add(new WarmStartHashSet(item.ObjectIndexA, item.ObjectIndexB), value.Item3);
                        //}

                }
            }

            var outputCollisionPoints = new List<CollisionPointStructure>(actualCollisionPoints);

            for (int i = 0; i < outputCollisionPoints.Count; i++)
            {
                var hashSet = new WarmStartHashSet(outputCollisionPoints[i].ObjectIndexA, outputCollisionPoints[i].ObjectIndexB);

                if (warmStartedCollisionPoints.TryGetValue(hashSet, out CollisionPointStructure cPoint))
                {
                    //if (cPoint.CollisionPointBase[0].CollisionPoints.Length >= outputCollisionPoints[i].CollisionPointBase[0].CollisionPoints.Length)
                    //{
                    CollisionPointStructure item = cPoint;
                    //item.CollisionPointBase[0].SetIntersection(outputCollisionPoints[i].CollisionPointBase[0].Intersection);
                    //item.CollisionPointBase[0].SetObjectDistance(outputCollisionPoints[i].CollisionPointBase[0].ObjectDistance);
                    //item.CollisionPointBase[0].SetIntersection(false);
                    //item.CollisionPointBase[0].SetObjectDistance(0.0);

                    int k = 0;
                    foreach (var value in item.CollisionPointBase[0].CollisionPoints)
                    {
                        value.RegularizeStartImpulseProperties(EngineParameters.WarmStartingValue);
                        value.SetIntersection(false);
                        value.SetDistance(0.0);
                        //value.SetNormal(outputCollisionPoints[i].CollisionPointBase[0].CollisionPoint.CollisionNormal);
                        k++;
                    }

                    outputCollisionPoints[i] = item;
                    //}
                }
            }

            return outputCollisionPoints;
        }

        #endregion

        #region Private Methods

        private void UpdatePersistentCollisionCounter(
            List<CollisionPointStructure> previousCollisionPoints,
            List<CollisionPointStructure> actualCollisionPoints,
            Dictionary<int, StabilizationValues> previousShapesProperties,
            IShape[] shapes)
        {
            double distTolerance = 0.001;
            double velTolerance = 0.2;

            if (actualCollisionPoints != null &&
                actualCollisionPoints.Count > 0)
            {
                //Check persistent collision
                var pCounter = collisionPointsCounterDictionary.ToArray();
                for (int i = 0; i < pCounter.Length; i++)
                {
                    var tupleVal = new Tuple<bool, int, CollisionPointStructure>(false, pCounter[i].Value.Item2, pCounter[i].Value.Item3);
                    pCounter[i] = new KeyValuePair<WarmStartHashSet, Tuple<bool, int, CollisionPointStructure>>(pCounter[i].Key, tupleVal);
                }
                collisionPointsCounterDictionary = pCounter.ToDictionary(x => x.Key, x => x.Value);

                for (int i = 0; i < actualCollisionPoints.Count; i++)
                {
                    WarmStartHashSet hash = new WarmStartHashSet(
                        actualCollisionPoints[i].ObjectIndexA,
                        actualCollisionPoints[i].ObjectIndexB);



                    if (collisionPointsCounterDictionary.TryGetValue(hash, out Tuple<bool, int, CollisionPointStructure> value))
                    {
                        if (previousShapesProperties.TryGetValue(hash.ID_A, out StabilizationValues objA) &&
                            previousShapesProperties.TryGetValue(hash.ID_B, out StabilizationValues objB))
                        {
                            //TODO Check position and velocity

                            var previousPoint = previousCollisionPoints.FirstOrDefault(x => x.ObjectIndexA == hash.ID_A && x.ObjectIndexB == hash.ID_B);
                            if (previousPoint == null)
                            {
                                previousPoint = previousCollisionPoints.FirstOrDefault(x => x.ObjectIndexA == hash.ID_B && x.ObjectIndexB == hash.ID_A);
                                hash = new WarmStartHashSet(
                                    actualCollisionPoints[i].ObjectIndexB,
                                    actualCollisionPoints[i].ObjectIndexA);
                            }

                            /*
                            var shapeA = shapes.First(x => x.ID == hash.ID_A);
                            var shapeB = shapes.First(x => x.ID == hash.ID_B);

                            var posDiffA = Vector3.Length(shapeA.Position - objA.Position);
                            var posDiffB = Vector3.Length(shapeB.Position - objB.Position);

                            var angDiffA = Quaternion.Length(shapeA.RotationStatus - objA.RotationStatus);
                            var angDiffB = Quaternion.Length(shapeB.RotationStatus - objB.RotationStatus);


                            if (previousPoint.CollisionPointBase[0].CollisionPoints.Length < 10)
                            {
                                var cPoint = actualCollisionPoints[i].CollisionPointBase[0].CollisionPoint;
                                cPoint.SetNormal(previousPoint.CollisionPointBase[0].CollisionPoint.CollisionNormal);
                                previousPoint.CollisionPointBase[0].AddCollisionPoint(cPoint);
                            }
                            */
                            collisionPointsCounterDictionary[hash] = new Tuple<bool, int, CollisionPointStructure>(true, value.Item2 + 1, previousPoint);

                            /*
                            if (posDiffA < distTolerance && posDiffB < distTolerance &&
                                angDiffA < distTolerance && angDiffB < distTolerance)
                            {
                                collisionPointsCounterDictionary[hash] = new Tuple<bool, int, CollisionPointStructure>(true, value.Item2 + 1, previousPoint);
                            }
                            else
                            {
                                if (collisionPointsCounterDictionary.ContainsKey(hash))
                                    collisionPointsCounterDictionary[hash] = new Tuple<bool, int, CollisionPointStructure>(true, 0, actualCollisionPoints[i]);
                                else
                                    collisionPointsCounterDictionary.Add(hash, new Tuple<bool, int, CollisionPointStructure>(true, 0, actualCollisionPoints[i]));
                            }
                            */

                        }
                    }
                    else
                    {


                        collisionPointsCounterDictionary.Add(hash, new Tuple<bool, int, CollisionPointStructure>(true, 0, actualCollisionPoints[i]));
                    }
                }

                var itemToDelete = collisionPointsCounterDictionary.ToArray().Where(x => !x.Value.Item1);

                foreach (var item in itemToDelete)
                    collisionPointsCounterDictionary.Remove(item.Key);
            }
            else
            {
                collisionPointsCounterDictionary = new Dictionary<WarmStartHashSet, Tuple<bool, int, CollisionPointStructure>>();
            }
        }

        #endregion
    }
}
