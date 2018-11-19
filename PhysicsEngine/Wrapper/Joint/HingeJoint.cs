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

using System;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper.Joint
{
    public sealed class HingeJoint : ICollisionJoint, IMapperJoint
    {

        #region Fields

        HingeConstraint hingeConstraint;

        #endregion

        #region Constructor

        public HingeJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3d startAnchorPosition,
            Vector3d hingeAxis,
            double restoreCoefficient,
            double springCoefficient)
        {
            hingeConstraint = new HingeConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPosition,
                hingeAxis,
                restoreCoefficient,
                springCoefficient);
        }

        #endregion

        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            hingeConstraint.AddTorque(torqueAxis1, torqueAxis2);
        }

        public Vector3d GetAnchorPosition()
        {
            return hingeConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return hingeConstraint;
        }

        public JointType GetJointType()
        {
            return hingeConstraint.GetJointType();
        }

        public int GetKeyIndex()
        {
            return hingeConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return hingeConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return hingeConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double? angularLimitMin, double? angularLimitMax)
        {
            hingeConstraint.SetAxis1AngularLimit(angularLimitMin, angularLimitMax);
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            hingeConstraint.SetAxis1Motor(speedValue, forceLimit);
        }

        public void SetAxis2AngularLimit(double? angularLimitMin, double? angularLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetAxis2Motor(double speedValue, double forceLimit)
        {
            throw new NotSupportedException();
        }

        public void SetLinearLimit(double linearLimitMin, double linearLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            hingeConstraint.SetErrorReductionParam(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            hingeConstraint.SetSpringCoefficient(springCoefficient);
        }

        #endregion
    }
}
