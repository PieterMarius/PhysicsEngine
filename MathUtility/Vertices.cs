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

