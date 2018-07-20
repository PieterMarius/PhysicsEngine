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
using SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition;
using SharpPhysicsEngine.ShapeDefinition;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.NonConvexDecomposition.Hierarchical_Tree
{
    internal class ConcaveHierarchicalTree: HierarchicalTree<Vertex3Index, AABB>
    {
        #region Fields

        private const double perturbationValue = 1E-5;

        #endregion

        #region Public Methods

        public List<ShapeDecompositionOutput> GetConvexShapeList(bool finalizeShape)
        {
            var convexShapeList = new List<ShapeDecompositionOutput>();
            GenerateConvexShapeList(ref convexShapeList);

            if (finalizeShape &&
                convexShapeList.Count > 0)
                FinalizeShape(ref convexShapeList);

            return convexShapeList;
        }

        public List<ShapeDecompositionOutput> GetIntersectedShape(
            AABB region,
            double distanceTolerance,
            bool finalizeShape)
        {
            var intersectedShapeList = new List<ShapeDecompositionOutput>();
            FindIntersectedShape(ref intersectedShapeList, region, distanceTolerance);

            if (finalizeShape &&
                intersectedShapeList.Count > 0)
                FinalizeShape(ref intersectedShapeList);

            return intersectedShapeList;
        }
                
        #endregion

        #region Private Methods

        private void GenerateConvexShapeList(ref List<ShapeDecompositionOutput> geometry)
        {
            if (Elements.Count > 0)
                geometry.Add(new ShapeDecompositionOutput(new HashSet<Vertex3Index>(Elements), Region));

            for (int i = 0; i < ChildNodes.Count; i++)
                ((ConcaveHierarchicalTree)ChildNodes[i]).GenerateConvexShapeList(ref geometry);
        }

        private void FindIntersectedShape(ref List<ShapeDecompositionOutput> geometry, AABB box, double distanceTolerance)
        {
            if (Region.Intersect(box, distanceTolerance))
            {
                if (Elements.Count > 0)
                    geometry.Add(new ShapeDecompositionOutput(new HashSet<Vertex3Index>(Elements), Region));

                for (int i = 0; i < ChildNodes.Count; i++)
                    ((ConcaveHierarchicalTree)ChildNodes[i]).FindIntersectedShape(ref geometry, box, distanceTolerance);
            }
        }

        private void FinalizeShape(ref List<ShapeDecompositionOutput> convexShapes)
        {
            Parallel.ForEach(
                convexShapes,
                new ParallelOptions { MaxDegreeOfParallelism = 1 },
                shape =>
                {
                    HashSet<Vertex3Index> bufVertex = new HashSet<Vertex3Index>();

                    foreach (var vertex in shape.Vertex3Idx)
                    {
                        foreach (var idx in vertex.Indexes)
                            bufVertex.Add(TotalElements[idx]);
                    }

                    shape.AddVertex3Index(bufVertex);

                    if (shape.Vertex3Idx.Count <= 3)
                    {
                        bufVertex.Clear();
                        for (int i = 0; i < 4 - shape.Vertex3Idx.Count; i++)
                        {
                            bufVertex.Add(new Vertex3Index(
                                shape.Vertex3Idx.First().Vector3 + Vector3.Random(-perturbationValue, perturbationValue),
                                new HashSet<int> { int.MaxValue - i },
                                0));
                        }
                        shape.AddVertex3Index(bufVertex);
                    }

                    var vertices = shape.Vertex3Idx.Select(x => x.Vector3).ToArray();
                    shape.SetRegion(AABB.GetGeometryAABB(vertices, null));
                });
        }

        #endregion
    }
}
