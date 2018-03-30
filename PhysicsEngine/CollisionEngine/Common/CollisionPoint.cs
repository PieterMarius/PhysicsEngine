﻿/******************************************************************************
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
using System.Collections.Generic;

namespace SharpPhysicsEngine.CollisionEngine
{
    public struct CollisionPoint
    {
        public readonly VertexProperties CollisionPointA;
        public readonly VertexProperties CollisionPointB;
        public readonly Vector3 CollisionNormal;
        public List<StartImpulseProperties> StartImpulseValue { get; private set; }

        #region Constructor

        public CollisionPoint(
            VertexProperties collisionPointA,
            VertexProperties collisionPointB,
            Vector3 collisionNormal)
        {
            CollisionPointA = collisionPointA;
            CollisionPointB = collisionPointB;
            CollisionNormal = collisionNormal;

            //Start Impulse Proprties respectively of Normal, Friction Axis1 and Friction Axis2
            StartImpulseValue = new List<StartImpulseProperties>()
            {
                new StartImpulseProperties(0.0),
                new StartImpulseProperties(0.0),
                new StartImpulseProperties(0.0),
            };
        }

        #endregion

        #region Public Methods

        public void RegularizeStartImpulseProperties(double regParam)
        {
            foreach (var item in StartImpulseValue)
                item.SetStartValue(item.StartImpulseValue * regParam);
            
        }

        #endregion
    }
}

