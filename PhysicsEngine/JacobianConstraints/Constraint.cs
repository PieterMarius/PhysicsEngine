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
using System.Collections.Generic;

namespace SharpPhysicsEngine
{
    internal abstract class Constraint: IConstraint, IConstraintBuilder
    {
        #region Fields

        protected Vector3 xVec = new Vector3(1.0, 0.0, 0.0);
        protected Vector3 xVecNeg = new Vector3(-1.0, 0.0, 0.0);
        protected Vector3 yVec = new Vector3(0.0, 1.0, 0.0);
        protected Vector3 yVecNeg = new Vector3(0.0, -1.0, 0.0);
        protected Vector3 zVec = new Vector3(0.0, 0.0, 1.0);
        protected Vector3 zVecNeg = new Vector3(0.0, 0.0, -1.0);

        protected IShape ShapeA;
        protected IShape ShapeB;
        protected int KeyIndex;
        protected double SpringCoefficient;
        protected double RestoreCoefficient;

        #endregion

        #region Constructor

        public Constraint(
            IShape shapeA,
            IShape shapeB,
            double restoreCoefficient,
            double springCoefficient)
        {
            ShapeA = shapeA;
            ShapeB = shapeB;
            KeyIndex = GetHashCode();
            SpringCoefficient = springCoefficient;
            RestoreCoefficient = restoreCoefficient;
        }
        
        #endregion

        #region Public Methods

        public int GetKeyIndex()
        {
            return KeyIndex;
        }

        public int GetObjectIndexA()
        {
            return ShapeA.ID;
        }

        public int GetObjectIndexB()
        {
            return ShapeB.ID;
        }

        public void SetRestoreCoefficient(double restoreCoefficient)
        {
            RestoreCoefficient = restoreCoefficient;
        }

        public void SetSpringCoefficient(double springCoefficient)
        {
            SpringCoefficient = springCoefficient;
        }

        public abstract List<JacobianConstraint> BuildJacobian(double? baumStabilization = null);
        public abstract Vector3 GetAnchorPosition();
        public abstract JointType GetJointType();
        public abstract void AddTorque(double torqueAxis1, double torqueAxis2);
        public abstract void SetAxis1AngularLimit(double angularLimitMin, double angularLimitMax);
        public abstract void SetAxis1Motor(double speedValue, double forceLimit);
        public abstract void SetAxis2AngularLimit(double angularLimitMin, double angularLimitMax);
        public abstract void SetAxis2Motor(double speedValue, double forceLimit);
        public abstract void SetLinearLimit(double linearLimitMin, double linearLimitMax);
              
        #endregion
    }
}
