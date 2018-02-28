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
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.Wrapper.Joint
{
    public sealed class SliderJoint : ICollisionJoint, IMapperJoint
    {
        #region Fields

        SliderConstraint sliderConstraint;

        #endregion

        #region Constructor

        public SliderJoint(
            ICollisionShape shapeA,
            ICollisionShape shapeB,
            Vector3 startAnchorPosition,
            Vector3 sliderAxis,
            double restoreCoefficient,
            double springCoefficient)
        {
            sliderConstraint = new SliderConstraint(
                ((IMapper)shapeA).GetShape(),
                ((IMapper)shapeB).GetShape(),
                startAnchorPosition,
                sliderAxis,
                restoreCoefficient,
                springCoefficient);
        }

        #endregion

        #region Public Methods

        public void AddTorque(double torqueAxis1, double torqueAxis2)
        {
            throw new NotSupportedException();
        }

        public Vector3 GetAnchorPosition()
        {
            return sliderConstraint.GetAnchorPosition();
        }

        IConstraint IMapperJoint.GetJoint()
        {
            return sliderConstraint;
        }

        public JointType GetJointType()
        {
            return JointType.Slider;
        }

        public int GetKeyIndex()
        {
            return sliderConstraint.GetKeyIndex();
        }

        public int GetObjectIndexA()
        {
            return sliderConstraint.GetObjectIndexA();
        }

        public int GetObjectIndexB()
        {
            return sliderConstraint.GetObjectIndexB();
        }

        public void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax)
        {
            throw new NotSupportedException();
        }

        public void SetAxis1Motor(double speedValue, double forceLimit)
        {
            sliderConstraint.SetAxis1Motor(speedValue, forceLimit);
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
            sliderConstraint.SetLinearLimit(linearLimitMin, linearLimitMax);
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            sliderConstraint.SetRestoreCoefficient(restoreCoefficient);
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            sliderConstraint.SetSpringCoefficient(springCoefficient);
        }

        #endregion
    }
}
