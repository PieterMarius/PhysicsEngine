using System;

namespace CollisionEngine
{
	public struct Box
	{
		#region Public Properties

		public readonly EndPoint[] Mmin;
		public readonly EndPoint[] Mmax;

		#endregion

		#region Constructor

		public Box (
			EndPoint[] mMin,
			EndPoint[] mMax)
		{
			this.Mmin = mMin;
			this.Mmax = mMax;
		}

		#endregion
	}
}

