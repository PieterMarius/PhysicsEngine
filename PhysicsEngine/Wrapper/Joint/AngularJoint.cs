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
    public sealed class AngularJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        AngularConstraint angularConstraint;

        #endregion

        #region Constructor

        public AngularJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3d startAnchorPos,
            Vector3d hingeAxis,
            Vector3d rotationAxis,
            double restoreCoefficient,
            double springCoefficientHingeAxis,
            double springCoefficientRotationAxis)
        {
            angularConstraint = new AngularConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPos,
                hingeAxis,
                rotationAxis,
                restoreCoefficient,
                springCoefficientHingeAxis,
                springCoefficientRotationAxis);
        }
        
        #endregion
        
        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotSupportedException();
        }

        public Vector3d GetAnchorPosition()
        {
            return angularConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return angularConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Angular;
        }

        public int GetKeyIndex()
        {
            return angularConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return angularConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return angularConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            throw new NotSupportedException();
        }

        public void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax)
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
            angularConstraint.SetErrorReductionParam(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            throw new NotSupportedException();
        }

        #endregion
    }
}
