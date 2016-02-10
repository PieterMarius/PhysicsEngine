using System;

namespace CollisionEngine
{
	public struct EndPoint
	{

		#region Public Properties

		public readonly int Index;
		public readonly float mValue;
		public readonly bool mIsMin;

		#endregion

		#region Constructor

		public EndPoint (
			int index,
			float value,
			bool isMin)
		{
			this.Index = index;
			this.mValue = value;
			this.mIsMin = isMin;
		}

		#endregion
	}
}

