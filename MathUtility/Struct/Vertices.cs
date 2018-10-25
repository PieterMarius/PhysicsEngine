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

namespace SharpEngineMathUtility
{
	public struct Vertices: IComparable
	{
		#region Properties

		public readonly Vector3d Vertex;
		public readonly double Angle;

		#endregion

		#region Constructor

		public Vertices (
			Vector3d vertex,
			double angle)
		{
			Vertex = vertex;
			Angle = angle;
		}

		#endregion

		#region IComparable implementation

		public int CompareTo (object obj)
		{
			if (obj is Vertices)
			{
				return CompareTo((Vertices)obj);
			}

			throw new ArgumentException ();
		}

		public int CompareTo(Vertices other)
		{
			if (Angle < other.Angle)
			{
				return -1;
			}

			if (Angle > other.Angle)
			{
				return 1;
			}

			return 0;
		}

		#endregion
	}
}

