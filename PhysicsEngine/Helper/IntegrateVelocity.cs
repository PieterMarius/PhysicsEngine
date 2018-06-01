using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.Helper
{
    internal sealed class IntegrateVelocity
    {
        #region Fields

        private static readonly double tolerance = 1E-50;
        private readonly PhysicsEngineParameters EngineParameters;

        #endregion 

        #region Constructor

        public IntegrateVelocity(PhysicsEngineParameters engineParameters)
        {
            EngineParameters = engineParameters;
        }

        #endregion Constructor

        #region Public Methods

        /// <summary>
        /// Update object velocity
        /// </summary>
        /// <param name="contact"></param>
        /// <param name="X"></param>
        public void UpdateVelocity(
            JacobianConstraint[] contact,
            double[] x)
        {
            var rangePartitioner = Partitioner.Create(0, contact.Length, Convert.ToInt32(contact.Length / EngineParameters.MaxThreadNumber) + 1);

            //Critical section variable
            var sync = new object();

            Parallel.ForEach(
                rangePartitioner,
                new ParallelOptions { MaxDegreeOfParallelism = EngineParameters.MaxThreadNumber },
                (range, loopState) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        if (Math.Abs(x[i]) > tolerance)
                        {
                            double impulse = x[i];

                            JacobianConstraint ct = contact[i];

                            if (ct.LinearComponentA.HasValue)
                                UpdateObjectVelocity(
                                    ct.ObjectA,
                                    ct.LinearComponentA.Value,
                                    ct.AngularComponentA,
                                    impulse,
                                    sync);
                            else
                                UpdateObjectVelocity(
                                    ct.ObjectA,
                                    ct.AngularComponentA,
                                    impulse,
                                    sync);

                            if (ct.LinearComponentB.HasValue)
                                UpdateObjectVelocity(
                                    ct.ObjectB,
                                    ct.LinearComponentB.Value,
                                    ct.AngularComponentB,
                                    impulse,
                                    sync);
                            else
                                UpdateObjectVelocity(
                                    ct.ObjectB,
                                    ct.AngularComponentB,
                                    impulse,
                                    sync);

                            if (ct.StartImpulse != null)
                                ct.StartImpulse.SetStartValue(impulse);
                        }
                    }
                });
        }

        #endregion

        #region Private Methods

        private void UpdateObjectVelocity(
            IShapeCommon simObj,
            Vector3 linearComponent,
            Vector3 angularComponent,
            double X,
            object sync)
        {
            if (!simObj.IsStatic)
            {
                Vector3 linearImpulse = X * linearComponent;
                Vector3 angularImpuse = X * angularComponent;

                Vector3 linearVelocity = linearImpulse *
                                         simObj.InverseMass;

                Vector3 angularVelocity = simObj.InertiaTensor *
                                          angularImpuse;

                //Critical Section
                lock (sync)
                {
                    simObj.SetLinearVelocity(simObj.LinearVelocity + linearVelocity);
                    simObj.SetAngularVelocity(simObj.AngularVelocity + angularVelocity);
                }
            }
        }

        private void UpdateObjectVelocity(
            IShapeCommon simObj,
            Vector3 angularComponent,
            double X,
            object sync)
        {
            if (!simObj.IsStatic)
            {
                Vector3 angularImpuse = X * angularComponent;

                Vector3 angularVelocity = simObj.InertiaTensor *
                                          angularImpuse;

                //Critical Section
                lock (sync)
                {
                    simObj.SetAngularVelocity(simObj.AngularVelocity + angularVelocity);
                }
            }
        }


        #endregion
    }
}
