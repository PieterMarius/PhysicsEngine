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
using System.Collections.Generic;
using System.Linq;

namespace SharpPhysicsEngine.K_Means
{
    internal sealed class KMeans
    {
        #region Fields

        public struct Centroid
        {
            public Vector3 Position;
            public List<Tuple<IKMeansInput, double>> Points;
        }

        #endregion

        #region Constructor

        public KMeans()
        { }

        #endregion

        #region Public Methods

        public Centroid[] Execute(IKMeansInput[] points, int nCluster)
        {
            var centroid = GetStartCentroid(points, nCluster);
            var newCentroid = Array.ConvertAll(centroid, x => x.Position);
            var oldCentroid = new Vector3[centroid.Length];
            
            while (CheckCentroid(oldCentroid, newCentroid))
            {
                Array.Copy(newCentroid, oldCentroid, newCentroid.Length);

                for (int i = 0; i < centroid.Length; i++)
                    centroid[i].Points = new List<Tuple<IKMeansInput, double>>();

                
                foreach (var item in points.Select((value, index) => new { value, index }))
                {
                    var centroidIndex = GetCentroidIndex(item.value, centroid);
                    centroid[centroidIndex.Item1].Points.Add(new Tuple<IKMeansInput, double>(item.value, centroidIndex.Item2));
                }
                
                for (int i = 0; i < centroid.Length; i++)
                {
                    newCentroid[i] = CalculateCentroid(centroid[i].Points);
                    centroid[i].Position = newCentroid[i];
                }
            }

            return centroid;
        }

        #endregion

        #region Private Methods

        private Centroid[] GetStartCentroid(IKMeansInput[] points, int nCluster)
        {
            var result = new Centroid[nCluster];
            int step = points.Length / nCluster;
            int index = 0;

            for (int i = 0; i < nCluster; i++)
            {
                result[i] = new Centroid() { Position = points[index].GetPointPosition(), Points = new List<Tuple<IKMeansInput, double>>() };
                index += step;
            }

            return result;
        }

        private Vector3 CalculateCentroid(List<Tuple<IKMeansInput, double>> centroidPoints)
        {
            var result = new Vector3();

            for (int i = 0; i < centroidPoints.Count; i++)
                result = result + centroidPoints[i].Item1.GetPointPosition();
            
            return result / centroidPoints.Count;
        }

        private Tuple<int, double> GetCentroidIndex(
            IKMeansInput point, 
            Centroid[] centroid)
        {
            double minDistance = double.MaxValue;
            int minCentroidIndex = 0;

            for (int i = 0; i < centroid.Length; i++)
            {
                Vector3 dist = point.GetPointPosition() - centroid[i].Position;
                double distance = dist.Dot(dist);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    minCentroidIndex = i;
                }
            }

            return new Tuple<int, double>(minCentroidIndex, minDistance);
        }

        private bool CheckCentroid(
            Vector3[] newCentroid, 
            Vector3[] oldCentroid)
        {
            for (int i = 0; i < newCentroid.Length; i++)
            {
                if (newCentroid[i] != oldCentroid[i])
                {
                    if (newCentroid[i].IsNaN() &&
                        oldCentroid[i].IsNaN())
                        continue;

                    return true;
                }
            }

            return false;
        }

        #endregion
    }
}
