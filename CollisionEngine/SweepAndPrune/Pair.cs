using System;

namespace CollisionEngine
{
	public struct Pair
	{
		#region Public Properties

		public readonly int IndexA;
		public readonly int IndexB;

		#endregion

		#region Constructor

		public Pair (
			int indexA,
			int indexB)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
		}

		#endregion
	}
}

