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

namespace SharpPhysicsEngine.Wrapper.Joint
{
    public interface ICollisionJoint
    {
        #region Get Methods
        Vector3 GetAnchorPosition();
        JointType GetJointType();
        int GetObjectIndexA();
        int GetObjectIndexB();
        int GetKeyIndex();

        #endregion

        #region Set Methods

        void SetLinearLimit(double linearLimitMin, double linearLimitMax);
        void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax);
        void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax);
        void SetAxis1Motor(double speedValue, double forceLimit);
        void SetAxis2Motor(double speedValue, double forceLimit);
        void AddTorque(double torqueAxis1, double torqueAxis2);
        void SetRestoreCoefficient(double restoreCoefficient);
        void SetSpringCoefficient(double springCoefficient);

        #endregion
    }
}
