using System;

namespace PhysicsEngineMathUtility
{
	public struct Vertices: IComparable
	{
		#region Properties

		public readonly Vector3 Vertex;
		public readonly double Angle;

		#endregion

		#region Constructor

		public Vertices (
			Vector3 vertex,
			double angle)
		{
			this.Vertex = vertex;
			this.Angle = angle;
		}

		#endregion

		#region IComparable implementation

		public int CompareTo (object obj)
		{
			if (obj is Vertices)
			{
				return this.CompareTo((Vertices)obj);
			}

			throw new ArgumentException ();
		}

		public int CompareTo(Vertices other)
		{
			if (this.Angle < other.Angle)
			{
				return -1;
			}

			if (this.Angle > other.Angle)
			{
				return 1;
			}

			return 0;
		}

		#endregion
	}
}

