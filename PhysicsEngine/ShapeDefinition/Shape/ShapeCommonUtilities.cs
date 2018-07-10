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
using System;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal static class ShapeCommonUtilities
    {
        #region Public Methods

        public static Vector3 CalculateCenterOfMass(
            Vector3[] vertices,
            TriangleMesh[] triangleMeshes,
            double mass)
        {
            var inertiaTensor = new InertiaTensorEngine(
                    vertices,
                    triangleMeshes,
                    mass,
                    false);

            return inertiaTensor.Execute().CenterOfMass;
        }

        public static InertiaTensorOutput GetInertiaTensor(
            Vector3[] vertexPosition,
            TriangleMesh[] triangleMeshes,
            Vector3 position,
            double mass)
        {
            var inertiaTensor = new InertiaTensorEngine(
                    vertexPosition,
                    triangleMeshes,
                    mass,
                    true);

            var inertiaTensorRes = inertiaTensor.Execute();

            Vector3 r = inertiaTensorRes.CenterOfMass - position;
            Matrix3x3 baseTensors= inertiaTensorRes.InertiaTensor +
                            (Matrix3x3.IdentityMatrix() * r.Dot(r) - Matrix3x3.OuterProduct(r, r)) *
                            mass;

            return new InertiaTensorOutput(baseTensors, inertiaTensorRes.CenterOfMass, inertiaTensorRes.DensityMass);
        }

        #endregion

        #region Private Methods

        

        #endregion
    }
}
