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
using SharpPhysicsEngine.CollisionEngine;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine
{
	internal static class ConstraintHelper
	{
        #region Public Methods

        public static JacobianConstraint[] FilterConstraints(
			JacobianConstraint[] list,
			ConstraintType typeA)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if (jc.Type == typeA)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] FilterConstraints(
			JacobianConstraint[] list,
			ConstraintType typeA,
			ConstraintType typeB)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if (jc.Type == typeA ||
					jc.Type == typeB)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] FindJointConstraints(JacobianConstraint[] list)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if (jc.Type != ConstraintType.Friction &&
					jc.Type != ConstraintType.Collision)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] FindConstraintsWithError(
			JacobianConstraint[] list,
			ConstraintType typeA,
			ConstraintType typeB)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if ((jc.Type == typeA ||
				    jc.Type == typeB) &&
				    Math.Abs(jc.CorrectionValue) > 1E-100)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static JacobianConstraint[] PruneConstraintsFromSoftJoint(
			JacobianConstraint[] list)
		{
			var result = new List<JacobianConstraint>();

			foreach (JacobianConstraint jc in list)
			{
				if (jc.Type != ConstraintType.SoftJoint)
					result.Add(jc);
			}

			return result.ToArray();
		}

		public static CollisionPointStructure Find(
			CollisionPointStructure[] collisionPoints,
			ContactIndex contactIndex)
		{
			foreach (CollisionPointStructure cps in collisionPoints)
			{
				if (cps.ObjectIndexA == contactIndex.IndexA &&
				    cps.ObjectIndexB == contactIndex.IndexB)
				{
					return cps;
				}
			}
			return null;
		}
        
        #endregion
    }
}

