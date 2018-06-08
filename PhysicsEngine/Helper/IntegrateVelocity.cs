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
